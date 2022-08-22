#include <Arduino.h>
#include <IRremote.h>
#include "TinyIRReceiver.hpp"

// Pins
int M1_extend_pin = 6;
int M1_retract_pin = 9;
int M2_extend_pin = 3;
int M2_retract_pin = 5;
int IR_pin = 2;
int fan_pin = 10;
int sw_top_extend = 7;
int sw_top_retract = 8;
int sw_bot_extend = 12;
int sw_bot_retract = 13;
int sw_door = 11;

// PWMs
int M1_extend_PWM = 0;
int M1_retract_PWM = 0;
int M2_extend_PWM = 0;
int M2_retract_PWM = 0;
int fan_PWM = 0;

//IR init
#define IR_INPUT_PIN 2
#if !defined(STR_HELPER)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif

volatile struct TinyIRReceiverCallbackDataStruct sCallbackData;

// global variables
bool retracting = false;
bool extending = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Initialising..\n");

  // Outputs
  pinMode(M1_extend_pin, OUTPUT);
  pinMode(M1_retract_pin, OUTPUT);
  pinMode(M2_extend_pin, OUTPUT);
  pinMode(M2_retract_pin, OUTPUT);
  pinMode(fan_pin, OUTPUT);
  // Inputs
  pinMode(sw_top_extend, INPUT_PULLUP); // high if switch activated
  pinMode(sw_top_retract, INPUT_PULLUP);
  pinMode(sw_bot_extend, INPUT_PULLUP);
  pinMode(sw_bot_retract, INPUT_PULLUP);
  pinMode(sw_door, INPUT_PULLUP);

  // Initialising outputs
  analogWrite(M1_extend_pin, 0);
  analogWrite(M1_retract_pin, 0);
  analogWrite(M2_extend_pin, 0);
  analogWrite(M2_retract_pin, 0);
  analogWrite(fan_pin, 0);

  // Initialising IR 
  Serial.begin(115200);
  #if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
      delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
  #endif
      // Just to know which program is running on my Arduino
  #if defined(ESP8266)
      Serial.println();
  #endif
      Serial.println(F("START " __FILE__ " from " __DATE__));
      if(!initPCIInterruptForTinyReceiver()){
          Serial.println(F("No interrupt available for pin " STR(IR_INPUT_PIN))); // optimized out by the compiler, if not required :-)
      }
      Serial.println(F("Ready to receive NEC IR signals at pin " STR(IR_INPUT_PIN)));

}

void loop() {
  if (sCallbackData.justWritten)
  {
    sCallbackData.justWritten = false;
    Serial.print(F("Address=0x"));
    Serial.print(sCallbackData.Address, HEX);
    Serial.print(F(" Command=0x"));
    Serial.print(sCallbackData.Command, HEX);
    if (sCallbackData.isRepeat)
    {
        Serial.print(F(" Repeat"));
    }
    Serial.println();
  }

  if(retracting && !extending)
  {
    int m1_retracted = digitalRead(sw_top_retract);
    int m2_retracted = digitalRead(sw_bot_retract);
    if(m1_retracted)
    {
      M1_retract_PWM = 0;
    }
    else
    {
      M1_retract_PWM = 180;
    }
    if(m2_retracted)
    {
      M2_retract_PWM = 0;
    }
    else
    {
      M2_retract_PWM = 180;
    }    
    M1_extend_PWM = 0;
    M2_extend_PWM = 0;

    if(m1_retracted && m2_retracted)
    {
      retracting = false;
    }
    
  }
  else if(!retracting && extending)
  {
    int m1_extended = digitalRead(sw_top_extend);
    int m2_extended = digitalRead(sw_bot_extend);
    if(m1_extended)
    {
      M1_extend_PWM = 0;
    }
    else
    {
      M1_extend_PWM = 180;
    }
    if(m2_extended)
    {
      M2_extend_PWM = 0;
    }
    else
    {
      M2_extend_PWM = 180;
    }
    M1_retract_PWM = 0;
    M2_retract_PWM = 0;

    if(m1_extended && m2_extended)
    {
      extending = false;
    }
  }
  else
  {
    M1_extend_PWM = 0;
    M1_retract_PWM = 0;
    M2_extend_PWM = 0;
    M2_retract_PWM = 0;
    fan_PWM = 0;
  }

  analogWrite(M1_extend_pin, M1_extend_PWM);
  analogWrite(M1_retract_pin, M1_retract_PWM);
  analogWrite(M2_extend_pin, M2_extend_PWM);
  analogWrite(M2_retract_pin, M2_retract_PWM);
  analogWrite(fan_pin, fan_PWM);

}


#if defined(ESP8266) || defined(ESP32)
void IRAM_ATTR handleReceivedTinyIRData(uint16_t aAddress, uint8_t aCommand, bool isRepeat)
#else
void handleReceivedTinyIRData(uint16_t aAddress, uint8_t aCommand, bool isRepeat)
#endif
{

#if defined(ARDUINO_ARCH_MBED) || defined(ESP32)
    // Copy data for main loop, this is the recommended way for handling a callback :-)
    sCallbackData.Address = aAddress;
    sCallbackData.Command = aCommand;
    sCallbackData.isRepeat = isRepeat;
    sCallbackData.justWritten = true;
#else
    /*
     * This is not allowed in ISR context for any kind of RTOS
     * For Mbed we get a kernel panic and "Error Message: Semaphore: 0x0, Not allowed in ISR context" for Serial.print()
     * for ESP32 we get a "Guru Meditation Error: Core  1 panic'ed" (we also have an RTOS running!)
     */
    // Print only very short output, since we are in an interrupt context and do not want to miss the next interrupts of the repeats coming soon
    Serial.print(F("A=0x"));
    Serial.print(aAddress, HEX);
    Serial.print(F(" C=0x"));
    Serial.print(aCommand, HEX);
    Serial.print(F(" R="));
    Serial.print(isRepeat);
    Serial.println();
#endif
}

/*
///// psudo code ////
////// attempt 2

retracting = ir sensor code
extending = ir sensor code

if retracting
  if sw_top_retract == pressed
    M1_PWM = 0
  if sw_bot_retract == pressed
    M2_PWM = 0

else if extending
  if sw_top_extend == pressed
    M1_PWM = 0
  if sw_bot_extend == pressed
    M2_PWM = 0

else
  all PWMs = 0


///Write PWM values
analogWrite(PWM, PWM)


*/