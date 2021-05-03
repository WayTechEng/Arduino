#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DS1307.h"


LiquidCrystal_I2C lcd(0x27, 2, 1, 0 , 4, 5, 6, 7, 3, POSITIVE);

int led = 6;
int sound_digital = 7;
int sound_analog = A0;
long int sum = 0;
int counter = 0;
int max_val = 0;
int arr[800] = {0};
int num = 150;
DS1307 clock;//define a object of DS1307 class

void setup(){
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  pinMode(sound_digital, INPUT);  

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Hello");
  lcd.setCursor( 0, 1 );
  delay(500);
  lcd.clear();
  lcd.print("SOUND DETECTOR");
  lcd.setCursor( 0, 1 );
  lcd.print("B.WAYMOUTH");
  delay(500);
}

void loop()
{
  for(int i = 0; i < num; i++)
  {
    int val_analog = analogRead(sound_analog);
    sum += val_analog;
  }
  int level = sum / num;
//  Serial.println(level);
  sum = 0;
  lcd.clear();
  lcd.print(level);
  delay(100);
}

//void loop(){
//  int val_digital = digitalRead(sound_digital);
//  int val_analog = analogRead(sound_analog);
//  sum += val_analog;
//  counter++;
//  if(val_analog > max_val)
//  {
//    max_val = val_analog;
//  }
//
//  if(counter >= 1000)
//  {
//    int avg = sum/counter;
////    Serial.println(val_analog);
////    Serial.println(max_val);
////    Serial.println(avg);
//    counter = 0;
//    sum = 0;
//    max_val = 0;
//  }
//
//  Serial.println(val_analog);
////  Serial.println("\t");
////  Serial.println(val_digital);
////
////  if (val_digital == HIGH)
////  {
////    digitalWrite (led, HIGH);
////    delay(3000);
////    }
////  else
////  {
////    digitalWrite (led, LOW);
////    }
//}
