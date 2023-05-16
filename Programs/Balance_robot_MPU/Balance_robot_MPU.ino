// #include <AFMotor.h>
#include "Wire.h"
#include <MPU6050_light.h>


MPU6050 mpu(Wire);
// AF_DCMotor motor(2);
float kp = 50;
float ki = 0;
float kd = 0;
float setpoint = 0;
double cum_err = 0;
float prev_err = 0;

int off_thresh_L = -20;
int off_thresh_R = 20;
float angles[10] = {0,0,0,0,0,0,0,0,0,0};
int arr_size = 10;
int max_pwm = 150;

int en1 = 2;
int en2 = 4;
int out1 = 5;
int out2 = 6;

const int dt_theory = 50;
float dt_actual = 20;
int counter = 0;
int n_counts = 20;
unsigned long t_s = millis();
unsigned long t_start = millis();

void setup()
{
  pinMode(out1, OUTPUT);
  pinMode(out2, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);

  analogWrite(out1, 0);
  analogWrite(out2, 0);
  digitalWrite(en1, 1);
  digitalWrite(en2, 1);
  
  Serial.begin(115200);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0)
  { 
  }
  // Serial.println(status);

  Serial.println(F("While calculating the offsets value, do not move the MPU6050 sensor"));
//  mpu.calcOffsets();
  // mpu.upsideDownMounting = true; // Comment on this line if MPU6050 is installed backwards
  
  Serial.println("Done!\n");
  t_s = millis();
  t_start = millis();
}

void loop()
{  
  mpu.update();
  if((millis() - t_start) >= dt_theory)
  {
    t_start = millis();
    counter++;
    float angle = mpu.getAngleY();
    int pwm_int = 0;

    // Push new angle into array, remove last one
    // for (int k = arr_size-1; k > 0; k--)
    // {        
    //   angles[k]=angles[k-1];
    // }
    // angles[0] = angle;

    // Serial.println("Angle list: ");
    // for (int i = 0; i < 10; i++)
    // {
    //   Serial.println(angles[i]);
    // }
    // Serial.println("Angle list done ! ");
    
    
    if(!((angle < off_thresh_L) || (angle > off_thresh_R)))
    {
      float error = setpoint + angle;
      cum_err += error * dt_actual;
      float pwm = error*kp + cum_err*ki + (error-prev_err)/dt_actual*kd;
      prev_err = error;
      
      if(pwm >= 0)
      {
        if(pwm >= max_pwm)
        {
          pwm_int = max_pwm;
        }
        else
        {
          pwm_int = int(pwm);
        }
//        digitalWrite(en1, 0);
//        digitalWrite(en2, 1);
        analogWrite(out1, pwm_int);
        analogWrite(out2, 0);
      }
      else if(pwm < 0)
      {
        if(pwm <= -max_pwm)
        {
          pwm_int = max_pwm;
        }
        else
        {
          pwm_int = int(-1*pwm);
        }
//        digitalWrite(en1, 1);
//        digitalWrite(en2, 0);
        analogWrite(out1, 0);
        analogWrite(out2, pwm_int);
      }
    }
    else
    {
//      digitalWrite(en1, 0);
//      digitalWrite(en2, 0);
      analogWrite(out1, 0);
      analogWrite(out2, 0);
    }
    if(counter == n_counts)
    {
      float t_t = millis() - t_s;
      float freq = n_counts / t_t;
      dt_actual = t_t / n_counts;
      // Serial.print("Time to complete 1000 cycles: ");
      // Serial.print(t_t);
      // Serial.print(" ms\n");
      // Serial.print("\nFrequency: ");
      // Serial.print(freq);
      // Serial.print(" Hz\n\n");
      // Serial.print("\nTime to complete 1 cycle: ");
      // Serial.print(dt_actual);
      // Serial.print(" ms\n");
      counter = 0;
      t_s = millis();
    }


//    Serial.print("PWM: ");
//    Serial.print(pwm_int);
      Serial.print("    Angle: ");
      Serial.print(angle);
    Serial.print("\n"); 
//       delay(50);

  }
  
}
