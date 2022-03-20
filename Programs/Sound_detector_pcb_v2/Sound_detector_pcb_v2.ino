/*
Arduino 2x16 LCD - Detect Buttons
modified on 18 Feb 2019
by Saeed Hosseini @ Electropeak
https://electropeak.com/learn/
*/
#include <LiquidCrystal.h>
//LCD pin to Arduino
const int pin_RS = 8; 
const int pin_EN = 9; 
const int pin_d4 = 4; 
const int pin_d5 = 5; 
const int pin_d6 = 6; 
const int pin_d7 = 7; 
const int pin_BL = 10; 
LiquidCrystal lcd( pin_RS,  pin_EN,  pin_d4,  pin_d5,  pin_d6,  pin_d7);

// Sound
int sound_analog = A1;
long int sum = 0;
int counter = 0;
int max_val = 0;
int arr[800] = {0};
int num = 350;

int numReadings = 9; // number of samples to take
int median = 0;      // median of the sorted samples
int readingNumber;   // counter for the sample array
byte i = 0;
byte j = 0;
byte position = 0;
int analogValues[9]; 

void bubbleSort();
int  averageArray();




void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("Distance:");
  lcd.setCursor(10,1);
}
void loop() {

  for (readingNumber = 0; readingNumber < numReadings; readingNumber++) 
  {
    for(int i = 0; i < num; i++)
    {
      int val_analog = analogRead(sound_analog);
      sum += val_analog;
    }
    int level = sum / num;
    sum = 0;
    //get the reading:
    analogValues[readingNumber] = level;
    // increment the counter:
    readingNumber++;
  }
  // sort the array using a bubble sort:
  bubbleSort();
  // get the middle element:
  median = analogValues[numReadings / 2]; 

  double med = double(median);
  double p = pow(med, -0.675);
  float distance = 147.02*p;
  
  
  lcd.setCursor(10,1);
//  int level = sum / num;
//  sum = 0;
  lcd.setCursor(10,1);
  lcd.print("      ");
  delay(1);
  lcd.setCursor(10,1);
  lcd.print(int(distance));
  lcd.print(" mm");
  delay(50);

}

void bubbleSort() 
{
  int out, in, swapper;
  for(out=0 ; out < numReadings; out++) {  // outer loop
    for(in=out; in<(numReadings-1); in++)  {  // inner loop
      if( analogValues[in] > analogValues[in+1] ) {   // out of order?
        // swap them:
        swapper = analogValues[in];
        analogValues [in] = analogValues[in+1];
        analogValues[in+1] = swapper;
      }
    }
  }
}
