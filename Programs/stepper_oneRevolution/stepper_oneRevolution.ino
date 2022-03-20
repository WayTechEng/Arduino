#include <Stepper.h>

const int button = 13;
const int ena = 2;
const int enb = 3;
const int stepsPerRevolution = 200;

Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

void setup() {
  // set the speed at 60 rpm:
  myStepper.setSpeed(160);
  // initialize the serial port:
  Serial.begin(9600);
  pinMode(button, INPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  digitalWrite(button, LOW);
}

void loop() {
  digitalWrite(ena, HIGH);
  digitalWrite(enb, HIGH);
  myStepper.step(stepsPerRevolution);

}
