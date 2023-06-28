#include <AFMotor.h>
#include <Wire.h>
#include <LSM6.h>

LSM6 imu;
AF_DCMotor motor(1);

char report[80];
float k = 0.2;

void setup()
{
  
  motor.setSpeed(100);
  motor.run(RELEASE);
  
  Serial.begin(9600);
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
}

void loop()
{
  imu.read();
//
//  snprintf(report, sizeof(report), "A: %6d %6d %6d    G: %6d %6d %6d",
//    imu.a.x, imu.a.y, imu.a.z,
//    imu.g.x, imu.g.y, imu.g.z);
//  Serial.println(report);
  float angle = float(imu.a.y);
  
  if(!((angle > 2800) || (angle < -2800)))
  {
    float pwm = angle * k;
    int pwm_int = 0;
    if((pwm > 255) || (pwm < -255))
    {
      pwm_int = int(255);
    }
    else if(pwm < 0)
    {
      pwm_int = int(-1*pwm);
    }
    else
    {
      pwm_int = int(pwm);
    }
    
    if(pwm >= 0)
    //  if(pwm < 0)
    {
      motor.run(FORWARD);
      motor.setSpeed(pwm_int);
    }
    else
    {
      motor.run(BACKWARD);
      motor.setSpeed(pwm_int);
    }
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print("        PWM: ");
    Serial.print(pwm_int);
    Serial.print("\n");  
      delay(50);
    //  motor.run(RELEASE);
  }
  else
  {
    motor.setSpeed(0);
  }
}
