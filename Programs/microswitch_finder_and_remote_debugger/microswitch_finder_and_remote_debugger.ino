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
int sw_bot_retract = 4;
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
      Serial.println("Disabling led output...");
      disableLEDFeedback();
      disableLEDFeedbackForReceive();
      disableLEDFeedbackForSend();
}

void loop() {
  if (sCallbackData.justWritten)
  {
    if(sCallbackData.Command == 0x4E)
    {
      Serial.println("You pressed the play buttton -> Extend mode activated");
    }
    else if(sCallbackData.Command == 0x11)
    {
      Serial.println("You pressed the stop button -> retract mode activated");
    }

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

  int top_extend = digitalRead(sw_top_extend);
  int bot_extend = digitalRead(sw_bot_extend);
  int top_retract = digitalRead(sw_top_retract);
  int bot_retract = digitalRead(sw_bot_retract);
  int door = digitalRead(sw_door);
  if(!top_extend)
  {
    Serial.println("top extend");
  }
  if(!bot_extend)
  {
    Serial.println("bot_extend");
  }
  if(!top_retract)
  {
    Serial.println("top_retract");
  }
  if(!bot_retract)
  {
    Serial.println("bot_retract");
  }
  if(!door)
  {
    Serial.println("door");
  }

  delay(500);

}

#if defined(ESP8266) || defined(ESP32)
void IRAM_ATTR handleReceivedTinyIRData(uint16_t aAddress, uint8_t aCommand, bool isRepeat)
#else
void handleReceivedTinyIRData(uint16_t aAddress, uint8_t aCommand, bool isRepeat)
#endif
{
  // Copy data for main loop, this is the recommended way for handling a callback :-)
  sCallbackData.Address = aAddress;
  sCallbackData.Command = aCommand;
  sCallbackData.isRepeat = isRepeat;
  sCallbackData.justWritten = true;
}