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
int rpm_sp = 90;
float Kp_rpm = 0.73;
float Ki_rpm = 0.1;
float Kd_rpm = 0.0; // 0.07
int I_err_rpm = 0;
int prev_err_rpm = 0;
int max_pwm = 255;

unsigned long dt_theory_us = 10000; // 10000us = 10ms
unsigned long dt_theory_ms = dt_theory_us/1000;
unsigned long dt_actual_us = 10000;
unsigned long dt_actual_ms = dt_actual_us/1000;
int counter = 0;
int n_counts = 20;
long encoder_pulse_counter = 0;
long direction = 1;
int pulses_arr[3] = {0,0,0};

unsigned long t_start_actual = micros();
unsigned long t_start_loop = micros();


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
  int pwm = 0;
  if( ((micros() - t_start_loop) >= dt_theory_us) )
  {    
    t_start_loop = micros();
    int encoder_pulse_counter_sum = 0;
    unsigned long rpm = 0;
    unsigned long rpm_avg = 0;
    int rpm_error = 0;
    int speed_avg = 0;

    for(int i=2;i>0;i--) // add values to rolling pulses-moving-average
    {
      pulses_arr[i] = pulses_arr[i-1];
      encoder_pulse_counter_sum += pulses_arr[i-1];
    }
    pulses_arr[0] = encoder_pulse_counter;
    encoder_pulse_counter_sum += encoder_pulse_counter;

    // rpm = 60000*encoder_pulse_counter/(dt_actual_us); // <---- (60000*encoder_pulse_counter/(dt_theory_ms)) /1008;
    rpm_avg = 60000/3*encoder_pulse_counter_sum/(dt_actual_us);

    int time_factor = 400000/dt_actual_us;  // equal to 40 if dt_actual=10000.  equal to 20 if dt_actual=20000. equal to 13 if dt_actual=30000 
    avg_speed = encoder_pulse_counter_sum * time_factor * 10/3;   // Adding factor of 10 to avoid rounding errors
    

    encoder_pulse_counter = 0;    
    
    if(true)
    {      
      rpm_error = rpm_sp - rpm_avg;
      I_err_rpm += (rpm_error * dt_actual_ms/5);
      // Serial.println("---------------------------");
      // Serial.println(rpm_error);
      // Serial.println(dt_actual_ms/2);
      // Serial.println(I_err_rpm);
      // Clamp I error, clamping at max_pwm/2 for now.
      if(I_err_rpm > max_pwm/2)
      {
        I_err_rpm = max_pwm/2;
        // Serial.print("HIGH: ");
        // Serial.println(I_err_rpm);
      }
      else if(I_err_rpm < -(max_pwm/2))
      {
        I_err_rpm = -max_pwm/2;
        // Serial.print("LOW: ");
        // Serial.println(I_err_rpm);
      }

      pwm = rpm_error*Kp_rpm + I_err_rpm*Ki_rpm + (rpm_error - prev_err_rpm)/dt_actual_us*Kd_rpm;
      prev_err_rpm = rpm_error;
      // Serial.println(pwm);
      
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
      
      else if(pwm < 0)
      {
        // if(pwm <= -max_pwm)
        // {
        //   pwm = max_pwm;
        // }
        // else
        // {
        //   pwm = -1*pwm;
        // }
        // Serial.println("\n--------------------------------------\n--------------------------------------\0 or below!\n--------------------------------------\n--------------------------------------");
        digitalWrite(en, LOW);
        analogWrite(out1, pwm);
        analogWrite(out2, 0);
        // analogWrite(out2, pwm);
      }
      counter++;         
    }
    else
    {
      digitalWrite(en, LOW);
      analogWrite(out1, 0);
      analogWrite(out2, 0);
    }

    Serial.print("RPM:");
    Serial.print(rpm_avg);
    Serial.print(",PWM:");
    Serial.print(pwm);
    Serial.print(",Setpoint:");
    Serial.print(rpm_sp);
    Serial.print(",Ki_rpm_dt:");
    Serial.print(I_err_rpm);
    Serial.print(",rpm_error:");
    Serial.println(rpm_error);
    
    // Time keeping
    if(counter == n_counts)
    {
      // float t_t = micros() - t_s;
      // float freq = n_counts / t_t;
      dt_actual_us = (micros() - t_start_actual) / n_counts;
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
          Kp_rpm -= .1;
          Serial.print("Kp: ");
          Serial.println(Kp_rpm);
          break;
        case 52: // 4
          Kp_rpm += .1;
          Serial.print("Kp: ");
          Serial.println(Kp_rpm);
          break;
        case 50: // 2
          Ki_rpm -= 0.01;
          Serial.print("Ki: ");
          Serial.println(Ki_rpm);
          break;
        case 53: // 5
          Ki_rpm += 0.01;
          Serial.print("Ki: ");
          Serial.println(Ki_rpm);
          break;
        case 51: // 3
          Kd_rpm -= .00001;
          Serial.print("Kd: ");
          Serial.println(Kd_rpm);
          break;
        case 54: // 6
          Kd_rpm += .00001;
          Serial.print("Kd: ");
          Serial.println(Kd_rpm);
          break;
        case 43: // "+"
          rpm_sp += 50;
          Serial.print("Setpoint: ");
          Serial.println(rpm_sp);
          break;
        case 45: // "-"
          rpm_sp -= 50;
          Serial.print("Setpoint: ");
          Serial.println(rpm_sp);
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
