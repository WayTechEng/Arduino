#include "PinChangeInterrupt.h"

#include <PWMFreak.h>
int pins_9_10 = 9;  // timer 1
int timer1_div = 1;
int en = 4;
int out1 = 5;
int out2 = 6;

unsigned long t_start_loop = micros(); //us
const int encoder_a = 7; // Pin 3
const int encoder_b = 8; // Pin 5
int encoder_pulse_counter = 0;
int direction = 1;
unsigned long dt_theory_us = 10000; // 10000us = 10ms
unsigned long dt_theory_ms = dt_theory_us/1000;
int pulses_arr[3] = {0,0,0};
// bool swap = false;
// int swap_counter = 0; 

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
  if((micros() - t_start_loop) >= dt_theory_us)
  {
    // Serial.print("\n------------------\n------------------\nHello\n------------------\n------------------\n");
    // if(swap)
    // {
    //   digitalWrite(en, LOW);
    //   analogWrite(out1, 100);
    //   analogWrite(out2, 0);
    // }
    // else
    // {
    //   digitalWrite(en, HIGH);
    //   analogWrite(out1, 100);
    //   analogWrite(out2, 0);
    // }
    digitalWrite(en, HIGH);
    analogWrite(out1, 10);
    analogWrite(out2, 0);

    int encoder_pulse_counter_sum = 0;
    for(int i=2;i>0;i--) // add values to rolling pulses-moving-average
    {
      pulses_arr[i] = pulses_arr[i-1];
      encoder_pulse_counter_sum += pulses_arr[i-1];
    }
    pulses_arr[0] = encoder_pulse_counter;
    encoder_pulse_counter_sum += encoder_pulse_counter;

    unsigned long rpm = 600*encoder_pulse_counter/(dt_theory_ms); // <---- (60000*encoder_pulse_counter/(dt_theory_ms)) /1008;
    unsigned long rpm_avg = 600/3*encoder_pulse_counter_sum/(dt_theory_ms);

    if(encoder_pulse_counter > 0)
    {
      unsigned long tick_per_10ms = 0;
      unsigned long tick_per_10ms_avg = 0;
      tick_per_10ms = encoder_pulse_counter/(dt_theory_ms/10);
      tick_per_10ms_avg = encoder_pulse_counter_sum/(3*dt_theory_ms/10);
      Serial.print("\nRPM (std, avg):            ");
      Serial.print(long(direction*rpm));
      Serial.print(",  ");
      Serial.println(long(direction*rpm_avg));
      // Serial.print("tick/us (std, avg):        ");
      // Serial.print(tick_per_10ms);
      // Serial.print(",  ");
      // Serial.println(tick_per_10ms_avg);
      Serial.print("Encoder pulses (std, avg): ");
      Serial.print(pulses_arr[0]);
      Serial.print(",  ");
      Serial.println(encoder_pulse_counter_sum/3);
    }
    encoder_pulse_counter = 0; // Clear variable just before counting again 
    t_start_loop = micros();
    
    // swap_counter++;
  }
  // if(swap_counter >= 40)
  // {
  //   swap_counter = 0;
  //   swap = !swap;
  // }
}