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
int speed_sp = 500;
int previous_speed_sp = 0;
int ref_speed_sp = 0;
// float Kp_speed = 0.24;
float Kp_speed = 0.10;
float Ki_speed = 0.47;
// float Ki_speed = 0;
// float Kp_speed = 0.109;
// float Ki_speed = 0.015;
float Kd_speed = 0.0; // 0.07
int I_err_speed = 0;
int prev_err_speed = 0;
int max_pwm = 255;
int oscil_min = 200;
int oscil_max = 1200;



unsigned long dt_theory_us = 20000; // 10000us = 10ms
unsigned long dt_theory_ms = dt_theory_us/1000;
unsigned long dt_actual_us = 20000;
unsigned long dt_actual_ms = dt_actual_us/1000;
int counter = 0;
int n_loops = 50;
unsigned long time_speed_change = 1000; // 1 second
long encoder_pulse_counter = 0;
long direction = 1;

// int pulses_arr[5] = {0,0,0,0,0};
// int pulses_arr[3] = {0,0,0};
int pulses_arr[1] = {0};
float average_scope = 1;

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
  int pwm = 0;
  if( ((micros() - t_start_loop) >= dt_theory_us) )
  {    
    t_start_loop = micros();
    int encoder_pulse_counter_sum = 0;
    unsigned long rpm = 0;
    unsigned long rpm_avg = 0;
    int rpm_error = 0;
    int speed_error = 0;
    double speed_avg = 0;
    int pwm_direction = 0; 

    for(int i=average_scope-1;i>0;i--) // add values to rolling pulses-moving-average
    {
      pulses_arr[i] = pulses_arr[i-1];
      encoder_pulse_counter_sum += pulses_arr[i-1];
    }
    pulses_arr[0] = encoder_pulse_counter;
    encoder_pulse_counter_sum += encoder_pulse_counter;

    // RPM_AVG  vs  SPEED_AVG
    //    2     :     40/3 = 13.33
    //    1     :     20/3 = 6.6667

    rpm_avg = 2*encoder_pulse_counter_sum;  // <-- rpm_avg = 60000/3*encoder_pulse_counter_sum/(dt_actual_us);

    double time_factor = 400000.0/dt_actual_us;  // equal to 40 if dt_actual=10000.  equal to 20 if dt_actual=20000. equal to 13 if dt_actual=30000 
    speed_avg = direction * encoder_pulse_counter_sum * 1.0 * time_factor / average_scope;

    //////// speed_avg = rpm_avg * 6.6667 ////////

    

    // Serial.print("\nRPM 2*PLS: ");
    // Serial.print(rpm_avg);
    // Serial.print("\nspeed_avg: ");
    // Serial.print(speed_avg);
    // Serial.print("\nRPM * 6.6667: ");

    // Serial.println(rpm_avg*6.6667);

    encoder_pulse_counter = 0;    
    
    if(true)
    {   
        
      speed_error = speed_sp - speed_avg;
      I_err_speed += (speed_error * dt_actual_ms/5);
      // Serial.println("---------------------------");
      // Serial.println(rpm_error);
      // Serial.println(dt_actual_ms/2);
      // Serial.println(I_err_rpm);
      // Clamp I error, clamping at max_pwm/2 for now.
      if(I_err_speed > max_pwm/2)
      {
        I_err_speed = max_pwm/2;
        // Serial.print("HIGH: ");
        // Serial.println(I_err_rpm);
      }
      else if(I_err_speed < -(max_pwm/2))
      {
        I_err_speed = -max_pwm/2;
        // Serial.print("LOW: ");
        // Serial.println(I_err_rpm);
      }

      pwm = speed_error*Kp_speed + I_err_speed*Ki_speed + (speed_error - prev_err_speed)/dt_actual_us*Kd_speed;


      if(previous_speed_sp != speed_sp)
      {
        ref_speed_sp = previous_speed_sp;
      }
      previous_speed_sp = speed_sp; 

      // Need a clause -- IF    current_speed_setpoint < previous_speed_setpoint AND they have the same sign
      //                  THEN  pwm direction must not change, pwm remain the same sign (but can go to zero..)
      if((speed_sp > 0) && (ref_speed_sp > 0))
      {
        
        if(ref_speed_sp > speed_sp)
        {
          // Serial.println("\nHere111\n");
          if(pwm < 0)
          {
            pwm = 0;
            // Serial.println("\nHere222\n");
          }
        }
      }
      else if((speed_sp < 0) && (ref_speed_sp < 0))
      {
        if(ref_speed_sp < speed_sp)
        {
          if(pwm > 0)
          {
            pwm = 0;
          }
        }
      }


      int corrected_pwm = pwm;
      if (pwm > 0)
      {
        pwm_direction = 1;
        if(pwm >= max_pwm)
        {
          corrected_pwm = max_pwm;
        }
        digitalWrite(en, HIGH);
        analogWrite(out1, corrected_pwm);
        analogWrite(out2, 0);
      }      
      else if(pwm < 0)
      {
        pwm_direction = -1;
        if(pwm <= -max_pwm)
        {
          corrected_pwm = max_pwm;
        }
        else
        {
          corrected_pwm = -1*pwm;
        }
        // Serial.println("\n--------------------------------------\n--------------------------------------\ or below!\n--------------------------------------\n--------------------------------------");
        // Serial.print("\nReverse direction: ");
        // Serial.print(pwm);
        digitalWrite(en, HIGH);
        analogWrite(out1, 0);
        analogWrite(out2, corrected_pwm);
        // analogWrite(out2, pwm);
        // Serial.println(pwm);
        // Serial.println(corrected_pwm);
      }
      else
      {
        digitalWrite(en, LOW);
        analogWrite(out1, 0);
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
    // Serial.print("RPM:");
    // Serial.print(rpm_avg);
    Serial.print(",Speed:");
    Serial.print(speed_avg);
    Serial.print(",PWM:");
    Serial.print(pwm);
    Serial.print(",Setpoint:");
    Serial.print(speed_sp);
    Serial.print(",Ki_speed_dt:");
    Serial.print(I_err_speed);
    Serial.print(",speed_error:");
    Serial.print(speed_error);
    Serial.print(",ref_speed_sp:");
    Serial.print(ref_speed_sp);
    // Serial.print(",Loop_speed:");
    // Serial.print(dt_actual_ms);
    // Serial.print(",PWM_DIRECTION:");
    // Serial.print(pwm_direction*100);
    Serial.print(",Kp_speed:");
    Serial.print(Kp_speed,2);
    Serial.print(",Ki_speed:");
    Serial.print(Ki_speed,3);
    Serial.print(",Kd_speed:");
    Serial.println(Kd_speed,5);
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


    // if((millis() - t_oscillation_timer) > time_speed_change)
    // {
    //   if(speed_sp > 500)
    //   {
    //     speed_sp = 500;
    //   }
    //   else
    //   {
    //     speed_sp = 1000;
    //   }
    //   t_oscillation_timer = millis();
    // }
    if((millis() - t_oscillation_timer) > time_speed_change)
    {
      if(speed_sp > oscil_min)
      {
        speed_sp = oscil_min;
      }
      else
      {
        speed_sp = oscil_max;
      }
      t_oscillation_timer = millis();
    }
    // if((millis() - t_oscillation_timer) > time_speed_change)
    // {
    //   if(speed_sp < -500)
    //   {
    //     speed_sp = -500;
    //   }
    //   else
    //   {
    //     speed_sp = -1000;
    //   }
    //   t_oscillation_timer = millis();
    // }

    // Serial.print("PWM: ");
    // Serial.println(pwm_int);

    // Serial.print("    Angle: ");
    // Serial.println(angle*100);
    // Serial.println(accum);

    prev_err_speed = speed_error;
    
  }
  
}
