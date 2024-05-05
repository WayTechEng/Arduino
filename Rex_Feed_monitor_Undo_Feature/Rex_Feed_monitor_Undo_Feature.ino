#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DS1307.h"

LiquidCrystal_I2C lcd(0x27, 2, 1, 0 , 4, 5, 6, 7, 3, POSITIVE);

// SDA - A4 , SCL - A5

const int buttonPin = 12;
volatile int buttonState = 0;
bool trigger = false;

int Pre_Sec;
DS1307 clock;//define a object of DS1307 class


void setup() {
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Hello");
  lcd.setCursor( 0, 1 );
  delay(1500);
  lcd.clear();
  lcd.print("REX HAS NOT BEEN");
  lcd.setCursor( 0, 1 );
  lcd.print("FED");
  pinMode(buttonPin, INPUT);

//  Serial.begin(9600);
  clock.begin();
  clock.fillByYMD(2017,8,18);//AUG 18,2017
  clock.fillByHMS(8,15,00);//12:00:00"
  clock.fillDayOfWeek(SAT);//Saturday
  clock.setTime();//write time to the RTC chip
  Pre_Sec=clock.second;
}

void loop()
{  
  buttonState = digitalRead(buttonPin);
  // Set LCD to FED and then check if holding condition is true
  if(buttonState == HIGH)
  {  
      lcd.clear();
      lcd.print("FED");
      trigger = true;
      delay(2000);
      buttonState = digitalRead(buttonPin);
      if(buttonState == HIGH)
      {
        lcd.clear();
        lcd.print("REX HAS NOT BEEN");
        lcd.setCursor(0, 1);
        lcd.print("FED");
        trigger = false;
        delay(5000);
      } 
  }
  
  if ((clock.hour == 4) && (clock.minute == 11) && (clock.second == 0))
  {
    lcd.clear();
    lcd.print("REX HAS NOT BEEN");
    lcd.setCursor( 0, 1 );
    lcd.print("FED");
    trigger = false;
  }
  
  printTime();  
}

void printTime()
{
  clock.getTime();
  if(clock.second!=Pre_Sec)
  {
      Pre_Sec=clock.second; 

      lcd.setCursor(0,1);
      if (trigger == false)
      {
        lcd.print("FED        ");
      }
      else
      {
        lcd.print("           ");
      }
      if(clock.hour <10 )
      {
        lcd.print("0");
      }
      lcd.print(clock.hour);    
      lcd.print(":");
      if(clock.minute < 10)
      {
        lcd.print("0");
      }
      lcd.print(clock.minute);

      /*
       * EXPERIMENTAL
       * 
       * 
       */
      // Priting empty spaces to eliminate any residual characters
      // Alternatively just add spaces before the time stamp to shift time stamp along.
      lcd.print("    ");
      
  }  
}