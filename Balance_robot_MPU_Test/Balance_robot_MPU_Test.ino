// #include <AFMotor.h>
#include "Wire.h"
#include <MPU6050_light.h>


MPU6050 mpu(Wire);
// AF_DCMotor motor(2);
float kp = 3;
float ki = 0;
float kd = 0;
float setpoint = 0;
double cum_err = 0;
float prev_err = 0;

int off_thresh_L = -45;
int off_thresh_R = 45;
float angles[10] = {0,0,0,0,0,0,0,0,0,0};
int arr_size = 10;
int max_pwm = 255;

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
    
    
    // if(!((angle < off_thresh_L) || (angle > off_thresh_R)))
    // {
    //   float error = setpoint + angle;
    //   cum_err += error * dt_actual;
    //   float pwm = error*kp + cum_err*ki + (error-prev_err)/dt_actual*kd;
    //   prev_err = error;
      
    //   if(pwm >= 0)
    //   {
    //     if(pwm >= max_pwm)
    //     {
    //       pwm_int = max_pwm;
    //     }
    //     else
    //     {
    //       pwm_int = int(pwm);
    //     }
    //     analogWrite(out1, pwm_int);
    //     analogWrite(out2, 0);
    //   }
    //   else if(pwm < 0)
    //   {
    //     if(pwm <= -max_pwm)
    //     {
    //       pwm_int = max_pwm;
    //     }
    //     else
    //     {
    //       pwm_int = int(-1*pwm);
    //     }
    //     analogWrite(out1, 0);
    //     analogWrite(out2, pwm_int);
    //   }
    // }
    // else
    // {
    //   analogWrite(out1, 0);
    //   analogWrite(out2, 0);
    // }

    Serial.print("PWM: ");
    Serial.print(pwm_int);
      Serial.print("    Angle: ");
      Serial.print(angle);
    Serial.print("\n"); 
      //  delay(50);

    if(angle >= 0)
    {
      analogWrite(out1, 0);
      analogWrite(out2, 50);
    }
    else
    {
      analogWrite(out1, 50);
      analogWrite(out2, 0);
    }
    
    

  }
  
}
