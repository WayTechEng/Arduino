#include <ELECHOUSE_CC1101.h> //Download it here: http://electronoobs.com/eng_arduino_ELECHOUSE_CC1101.php
const int button = 0; //buzzer to arduino pin 9
const int buzzer_pos = 7;
int press_counter = 0;
int threshold = 7;


byte TX_buffer[1]={0};

void setup()
{
  pinMode(button, INPUT_PULLUP);
  pinMode(buzzer_pos, OUTPUT);
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  ELECHOUSE_cc1101.Init();
  TX_buffer[0]=0;
}

void loop()
{
  int buttonVal = digitalRead(button);
  if (buttonVal == HIGH) 
  {
    digitalWrite(13, LOW);
  } 
  else 
  {
    press_counter++;

    if(press_counter <= threshold)
    {
      tone(buzzer_pos, 500); // Send 1KHz sound signal...
      delay(1100);        // ...for 1 sec
      noTone(buzzer_pos);     // Stop sound...
      delay(100);        // ...for 1sec
      TX_buffer[0] = 1;
      ELECHOUSE_cc1101.SendData(TX_buffer,1);
      delay(1000);
      Serial.println("Pressed button less than 5 times");
    }
    else
    {
      out();
      Serial.println("Pressed button more than 5 times");
    }
    

    
    
    
  }
}

void out()
{
  for (int i = 0; i < 10; i++)
    {
      tone(buzzer_pos, 500); // Send 1KHz sound signal...
      delay(100);        // ...for 1 sec
      noTone(buzzer_pos);     // Stop sound...
      delay(100);        // ...for 1sec
    }
}

//  TX_buffer[0] = 1;
//  ELECHOUSE_cc1101.SendData(TX_buffer,size);
//  delay(100);
//  TX_buffer[0] = 0;
//  ELECHOUSE_cc1101.SendData(TX_buffer,size);
//  delay(100);
//  TX_buffer[0] = 2;
//  ELECHOUSE_cc1101.SendData(TX_buffer,size);
//  delay(100);
