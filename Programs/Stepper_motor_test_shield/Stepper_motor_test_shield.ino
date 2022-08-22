#include <AFMotor.h>

const int steps_per_rev = 200; //Set to 200 for NIMA 17 and set to 48 for 28BYJ-48

// Motor connected with port1 (M1 and M2)
AF_Stepper motor(steps_per_rev, 1);

void setup() {
  Serial.begin(115200);
  motor.setSpeed(50);  
}

void loop() {
//  Serial.println("Single coil");
//  motor.step(400, FORWARD, SINGLE); 
//  motor.step(100, BACKWARD, SINGLE); 

  Serial.println("Double coil");
  motor.step(400, FORWARD, DOUBLE);
  delay(1000);
  motor.step(400, BACKWARD, DOUBLE);
  delay(1000);00
//  Serial.println("Interleave");
//  motor.step(400, FORWARD, INTERLEAVE); 
//  motor.step(400, BACKWARD, INTERLEAVE); 
//
//  Serial.println("Micrsostep");
//  motor.step(400, FORWARD, MICROSTEP); 
//  motor.step(400, BACKWARD, MICROSTEP); 
}
