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

float time_till_full_speed = 4000;
unsigned long start_timer;
unsigned long reversal_timer;
int reversal_lag_duration = 2000;
bool instant_reversal_avoided = false;
  bool avoid_instant_reversal = false;

int M1_max_PWM = 150;
int M2_max_PWM = 200;

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
    bool init_extend = sCallbackData.Command == 0x4E;
    bool init_retract = sCallbackData.Command == 0x11;

    if(avoid_instant_reversal)
    {
      Serial.println(millis() - reversal_timer);
      if((millis() - reversal_timer) > reversal_lag_duration)
      {
        Serial.println("Reversal time-delay reached");
        avoid_instant_reversal = false;
        if(init_extend)
        {
          Serial.println("You pressed the play buttton -> Extend mode activated");
          if(!extending)
          {
            start_timer = millis();
          }
          extending = true;
        }
        else if(init_retract)
        {
          Serial.println("You pressed the stop button -> retract mode activated");
          if(!retracting)
          {
            start_timer = millis();
          }
          retracting = true;
        }
      }
    }    
    else if(retracting && init_extend)
    {
      reversal_timer = millis();
      avoid_instant_reversal = true;
      extending = true;
      Serial.println("Attemped to EXTEND while RETRACTING");
    }
    else if(extending && init_retract)
    {
      reversal_timer = millis();
      avoid_instant_reversal = true;
      retracting = true;
      Serial.println("Attemped to RETRACT while EXTENDING");
    }
    else if(init_extend)
    {
      Serial.println("You pressed the play buttton -> Extend mode activated");
      if(!extending)
      {
        start_timer = millis();
      }
      extending = true;
      
    }
    else if(init_retract)
    {
      Serial.println("You pressed the stop button -> retract mode activated");
      if(!retracting)
      {
        start_timer = millis();
      }
      retracting = true;
      
    }
  }

  int door_open = digitalRead(sw_door);

  if(retracting && !extending && !door_open)
  {
    unsigned long time_elapsed = millis() - start_timer;
    int m1_retracted = digitalRead(sw_top_retract);
    int m2_retracted = digitalRead(sw_bot_retract);
    if(m1_retracted)
    {
      M1_retract_PWM = 0;
    }
    else
    {
      M1_retract_PWM = M1_max_PWM * float(time_elapsed)/time_till_full_speed;
      if(M1_retract_PWM > M1_max_PWM)
      {
        M1_retract_PWM = M1_max_PWM;
      }
    }
    if(m2_retracted)
    {
      M2_retract_PWM = 0;
    }
    else
    {
      M2_retract_PWM = M2_max_PWM * float(time_elapsed)/time_till_full_speed;
      if(M2_retract_PWM > M2_max_PWM)
      {
        M2_retract_PWM = M2_max_PWM;
      }
    }    
    M1_extend_PWM = 0;
    M2_extend_PWM = 0;

    if(m1_retracted && m2_retracted)
    {
      retracting = false;
      // Serial.println("At home position");
    }
    
  }
  else if(!retracting && extending && !door_open)
  {
    unsigned long time_elapsed = millis() - start_timer;
    int m1_extended = digitalRead(sw_top_extend);
    int m2_extended = digitalRead(sw_bot_extend);
    if(m1_extended)
    {
      M1_extend_PWM = 0;
    }
    else
    {
      M1_extend_PWM = M1_max_PWM * float(time_elapsed)/time_till_full_speed;
      if(M1_extend_PWM > M1_max_PWM)
      {
        M1_extend_PWM = M1_max_PWM;
      }
    }
    if(m2_extended)
    {
      M2_extend_PWM = 0;
    }
    else
    {
      M2_extend_PWM = M2_max_PWM * float(time_elapsed)/time_till_full_speed;
      if(M2_extend_PWM > M2_max_PWM)
      {
        M2_extend_PWM = M2_max_PWM;
      }
    }
    M1_retract_PWM = 0;
    M2_retract_PWM = 0;

    if(m1_extended && m2_extended)
    {
      extending = false;
      // Serial.println("At extended position");
    }
  }
  else
  {
    M1_extend_PWM = 0;
    M1_retract_PWM = 0;
    M2_extend_PWM = 0;
    M2_retract_PWM = 0;
    fan_PWM = 0;
    retracting = false;
    extending = false;
    // if(door_open)
    // {
    //   Serial.println("Warning.. Door open!");
    // }
    // Serial.println("Setting PWMs to zero");
    delay(50);
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
  // don't do anything intensive in interupt
  sCallbackData.Address = aAddress;
  sCallbackData.Command = aCommand;
  sCallbackData.isRepeat = isRepeat;
  sCallbackData.justWritten = true;
}