#include "PinChangeInterrupt.h"

#include <PWMFreak.h>
int pins_9_10 = 9;  // timer 1
int timer1_div = 1;
int en = 4;
int out1 = 5;
int out2 = 6;

unsigned long t_start = micros(); //us
const int encoder_a = 7; // Pin 3
const int encoder_b = 8; // Pin 5
int encoder_pulse_counter = 0;
int direction = 1;
unsigned int dt_theory = 50000; // 10000us = 10ms
bool swap = false;
int swap_counter = 0; 

void encoderPinChangeA()
{
    encoder_pulse_counter += 1;
    direction = digitalRead(encoder_a) == digitalRead(encoder_b) ? -1 : 1;
}

void encoderPinChangeB()
{
    encoder_pulse_counter += 1;
    direction = digitalRead(encoder_a) != digitalRead(encoder_b) ? -1 : 1;
}

void setup() 
{
    Serial.begin(115200);
    pinMode(encoder_a, INPUT_PULLUP);
    pinMode(encoder_b, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(encoder_a), encoderPinChangeA, CHANGE);
    attachPCINT(digitalPinToPCINT(encoder_b), encoderPinChangeB, CHANGE);
    
    pinMode(out1, OUTPUT);
    pinMode(out2, OUTPUT);
    pinMode(en, OUTPUT);
    setPwmFrequency(pins_9_10, timer1_div); // timer 1

}

void loop()
{
  if((micros() - t_start) >= dt_theory)
  {
    if(swap)
    {
      digitalWrite(en, LOW);
      analogWrite(out1, 100);
      analogWrite(out2, 0);
    }
    else
    {
      digitalWrite(en, HIGH);
      analogWrite(out1, 100);
      analogWrite(out2, 0);
    }
    
    unsigned long rpm = (60000*encoder_pulse_counter/(dt_theory/1000)) /1008;
    unsigned long tick_per_10ms = 0;
    if(encoder_pulse_counter > 0)
    {
      tick_per_10ms = encoder_pulse_counter/(dt_theory/10000);
      Serial.print("RPM: ");
      Serial.println(long(direction*rpm));
      Serial.print("tick/us: ");
      Serial.println(tick_per_10ms);
      Serial.print("Encoder pulses: ");
      Serial.println(encoder_pulse_counter);
    }
    encoder_pulse_counter = 0; // Clear variable just before counting again 
    t_start = micros();
    swap_counter++;
  }
  if(swap_counter >= 40)
  {
    swap_counter = 0;
    swap = !swap;
  }
}