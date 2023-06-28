#include <ELECHOUSE_CC1101.h> //Download it here: http://electronoobs.com/eng_arduino_ELECHOUSE_CC1101.php
int received_number = 0;

const int A = 9;
const int B = 8;
const int C = 7;
const int D = 6;
int steps_f = 360;
int steps_r = 50;
int dt = 2;

 void setup()
{
  Serial.begin(9600);
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.SetReceive();

  // Stepper pins
  pinMode(A, OUTPUT); 
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);
}

byte RX_buffer[11]={0};
byte size,i,flag;

void clockwiserotate() { //revolve clockwise
  step1();
  step2();
  step3();
  step4();
  step5();
  step6();
  step7();
  step8();
}
void counterclockwiserotate() { //revolve counterclockwise
  step1();
  step7();
  step6();
  step5();
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
  digitalWrite(A, HIGH);
  digitalWrite(B, HIGH);
  digitalWrite(C, LOW);
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
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH);
  digitalWrite(D, LOW);
  delay(dt);
}
void step5(){
  digitalWrite(A, LOW);
  digitalWrite(B, LOW);
  digitalWrite(C, HIGH);
  digitalWrite(D, LOW);
  delay(dt);
}
void step6(){
  digitalWrite(A, LOW);
  digitalWrite(B, LOW);
  digitalWrite(C, HIGH);
  digitalWrite(D, HIGH);
  delay(dt);
}
void step7(){
  digitalWrite(A, LOW);
  digitalWrite(B, LOW);
  digitalWrite(C, LOW);
  digitalWrite(D, HIGH);
  delay(dt);
}
void step8(){
  digitalWrite(A, HIGH);
  digitalWrite(B, LOW);
  digitalWrite(C, LOW);
  digitalWrite(D, HIGH);
  delay(dt);
}

void loop()
{
  if(ELECHOUSE_cc1101.CheckReceiveFlag())
  {
    size=ELECHOUSE_cc1101.ReceiveData(RX_buffer);
    for(i=0;i<1;i++)
    {
      received_number = RX_buffer[i];
      Serial.println(received_number);  

      if(received_number == 1)
      {
        int i=0;
        for(i=0;i<steps_f;i++)
        {
          clockwiserotate();
        }  
        delay(200);
        i=0;
        for(i=0;i<steps_r;i++)
        {
          counterclockwiserotate();
        }  
      }
    }
    ELECHOUSE_cc1101.SetReceive();
  }

  digitalWrite(A, LOW);
  digitalWrite(B, LOW);
  digitalWrite(C, LOW);
  digitalWrite(D, LOW);
}
