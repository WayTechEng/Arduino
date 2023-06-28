/*Note: This code has been tested under Arduino IDE 1.8.3
 Stepper Motor Driving
 
 This program drives a unipolar stepper motor. 
 The motor is attached to digital pins 8 - 11 of Arduino.
 
 The motor should revolve one revolution in one direction, then
 one revolution in the other direction.  
  */

const int A = 8;
const int B = 9;
const int C = 10;
const int D = 11;
int steps = 300;
int dt = 2;
void setup() {                
  // initialize the digital pin as an output.
  pinMode(A, OUTPUT); 
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);
}
void loop() {
  int i=0;
  //revolve one revolution clockwise
  for(i=0;i<steps;i++){
  clockwiserotate();}  
//  delay(500);              // wait for a second
// //revolve one revolution counterclockwise
// for(i=0;i<512;i++){
//  counterclockwiserotate();}
//  delay(1000);              // wait for a second
}
void clockwiserotate() { //revolve clockwise
  step1();
  step2();
  step3();
  step4();
}
void counterclockwiserotate() { //revolve counterclockwise
  step4();
  step3();
  step2();
  step1();
}
void step1(){
  digitalWrite(A, HIGH);
  digitalWrite(B, LOW);
  digitalWrite(C, LOW);
  digitalWrite(D, LOW);
  delay(dt);
}
void step2(){
  digitalWrite(A, LOW);
  digitalWrite(B, LOW);
  digitalWrite(C, HIGH);
  digitalWrite(D, LOW);
  delay(dt);
}
void step3(){
  digitalWrite(A, LOW);
  digitalWrite(B, HIGH);
  digitalWrite(C, LOW);
  digitalWrite(D, LOW);
  delay(dt);
}
void step4(){
  digitalWrite(A, LOW);
  digitalWrite(B, LOW);
  digitalWrite(C, LOW);
  digitalWrite(D, HIGH);
  delay(dt);
}
