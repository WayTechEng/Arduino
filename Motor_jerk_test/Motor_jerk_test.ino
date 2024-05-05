// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include <PWMFreak.h>
#include "PinChangeInterrupt.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Pin declarations

int en = 4;
int out1 = 5;
int out2 = 6;
int encoder_a = 7; // Pin 3
int encoder_b = 8; // Pin 5

// https://nerdytechy.com/how-to-change-the-pwm-frequency-of-arduino/
// https://www.etechnophiles.com/change-frequency-pwm-pins-arduino-uno/
int pins_9_10 = 9;  // timer 1
int timer1_div = 1;

// PID elements
int speed_sp = 1000;
float Kp_speed = 0.18;
float Ki_speed = 0.08;
// float Ki_speed = 0;
// float Kp_speed = 0.109;
// float Ki_speed = 0.015;
float Kd_speed = 0.0; // 0.07
int I_err_speed = 0;
int prev_err_speed = 0;
int max_pwm = 255;
int pwm = 50;


unsigned long dt_theory_us = 10000; // 10000us = 10ms
unsigned long dt_theory_ms = dt_theory_us/1000;
unsigned long dt_actual_us = 10000;
unsigned long dt_actual_ms = dt_actual_us/1000;
int counter = 0;
int n_loops = 50;
unsigned long time_speed_change = 9; // 1 second
long encoder_pulse_counter = 0;
long direction = 1;

int pulses_arr[5] = {0,0,0,0,0};
float average_scope = 5;

unsigned long t_start_actual = micros();
unsigned long t_start_loop = micros();
unsigned long t_oscillation_timer = millis();  // milliseconds


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

void setup() {
    Serial.begin(115200);

    pinMode(out1, OUTPUT);
    pinMode(out2, OUTPUT);
    pinMode(en, OUTPUT);

    analogWrite(out1, 0);
    analogWrite(out2, 0);
    digitalWrite(en, 1);

    setPwmFrequency(pins_9_10, timer1_div); // timer 1

    pinMode(encoder_a, INPUT_PULLUP);
    pinMode(encoder_b, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(encoder_a), encoderPinChangeA, CHANGE);
    attachPCINT(digitalPinToPCINT(encoder_b), encoderPinChangeB, CHANGE);  
}


void loop()
{  
  
  if( ((micros() - t_start_loop) >= dt_theory_us) )
  {    
    t_start_loop = micros();
    int encoder_pulse_counter_sum = 0;
    unsigned long rpm = 0;
    unsigned long rpm_avg = 0;
    int rpm_error = 0;
    int speed_error = 0;
    double speed_avg = 0;

    if(true)
    { 
      if((millis() - t_oscillation_timer) > time_speed_change)
      {
        if(pwm < 60)
        {
          pwm = 70;
        }
        else
        {
          pwm = 50;
        }
        t_oscillation_timer = millis();
      }

      if (pwm > 0)
      {
        if(pwm >= max_pwm)
        {
          pwm = max_pwm;
        }
        digitalWrite(en, HIGH);
        analogWrite(out1, pwm);
        analogWrite(out2, 0);
      }
      counter++;         
    }
    else
    {
      digitalWrite(en, LOW);
      analogWrite(out1, 0);
      analogWrite(out2, 0);
    }

///////////////////////////////////////////////////
//      Serial plotter
///////////////////////////////////////////////////
    Serial.print("RPM:");
    Serial.print(rpm_avg);
    Serial.print(",Speed:");
    Serial.print(speed_avg);
    Serial.print(",PWM:");
    Serial.print(pwm);
    Serial.print(",Setpoint:");
    Serial.print(speed_sp);
    Serial.print(",Ki_speed_dt:");
    Serial.print(I_err_speed);
    Serial.print(",speed_error:");
    Serial.println(speed_error);
///////////////////////////////////////////////////

    // Time keeping
    if(counter == n_loops)
    {
      // float t_t = micros() - t_s;
      // float freq = n_loops / t_t;
      dt_actual_us = (micros() - t_start_actual) / n_loops;
      dt_actual_ms = dt_actual_us/1000;
      // Serial.print("Time to complete 1000 cycles: ");
      // Serial.print(t_t);
      // Serial.print(" ms\n");
      // Serial.print("\nFrequency: ");
      // Serial.print(freq);
      // Serial.print(" Hz\n\n");
      // Serial.print("Time to complete 1 cycle: ");
      // Serial.println(dt_actual_us);
      // Serial.println(" ms");
      // Serial.println(rpm_error);
      // Serial.println(angle);
      // Serial.println(accum);
      // Serial.println();
      // Serial.println(encoder_pulse_counter);
      // Serial.print("RPM:");
      // Serial.print(rpm_avg);
      // Serial.print(",Setpoint:");
      // Serial.print(rpm_sp);
      // Serial.print(",PWM:");
      // Serial.print(pwm);
      // Serial.println();
      counter = 0;
      t_start_actual = micros();
    }

    /// Keyboard inputs
    if(Serial.available() > 0)
    {
      int s_input = Serial.read();
      // Serial.println(s_input);
      switch(s_input)
      {
        case 49: // 1
          Kp_speed -= .01;
          Serial.print("Kp: ");
          Serial.println(Kp_speed);
          break;
        case 52: // 4
          Kp_speed += .01;
          Serial.print("Kp: ");
          Serial.println(Kp_speed);
          break;
        case 50: // 2
          Ki_speed -= 0.001;
          Serial.print("Ki: ");
          Serial.println(Ki_speed);
          break;
        case 53: // 5
          Ki_speed += 0.001;
          Serial.print("Ki: ");
          Serial.println(Ki_speed);
          break;
        case 51: // 3
          Kd_speed -= .00001;
          Serial.print("Kd: ");
          Serial.println(Kd_speed);
          break;
        case 54: // 6
          Kd_speed += .00001;
          Serial.print("Kd: ");
          Serial.println(Kd_speed);
          break;
        case 43: // "+"
          speed_sp += 50;
          Serial.print("Setpoint: ");
          Serial.println(speed_sp);
          break;
        case 45: // "-"
          speed_sp -= 50;
          Serial.print("Setpoint: ");
          Serial.println(speed_sp);
          break;
      }
    }



    // Serial.print("PWM: ");
    // Serial.println(pwm_int);

    // Serial.print("    Angle: ");
    // Serial.println(angle*100);
    // Serial.println(accum);

  }
  
}
