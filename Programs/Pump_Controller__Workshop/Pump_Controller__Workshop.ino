#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DS1307.h"

int charging_led = 7;
int charging_enable = 4;
int bat_voltage_pin = A0;
int raw_val = 0;
double max_voltage = 12.8;

LiquidCrystal_I2C lcd(0x27, 2, 1, 0 , 4, 5, 6, 7, 3, POSITIVE);

void setup() {
  // put your setup code here, to run once:
  pinMode(charging_led, OUTPUT);
  pinMode(charging_enable, OUTPUT);

  Serial.begin(9600);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Hello");
  digitalWrite(charging_led, HIGH);
  digitalWrite(charging_enable, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  raw_val = analogRead(bat_voltage_pin);
  double bat_voltage = max_voltage * (raw_val/1023.0);
  double test = 823.0/1023.0;
  
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("B_VOLTS: ");
  lcd.print(bat_voltage);

//  Serial.println(raw_val);
//  printDouble(test, 3);
//  Serial.println(".. .. ");
//  Serial.println("");
  

//  digitalWrite(charging_led, !digitalRead(charging_led));
//  digitalWrite(charging_enable, !digitalRead(charging_enable));
  
  delay(1000);
  
}


void printDouble( double val, byte precision){
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)

  Serial.println("");
  Serial.println("");
  Serial.print (int(val));  //prints the int part
  if( precision > 0) {
    Serial.print("."); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision -1;
    while(precision--)
       mult *=10;
       
    if(val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val)- val ) * mult;
    unsigned long frac1 = frac;
    while( frac1 /= 10 )
      padding--;
    while(  padding--)
      Serial.print("0");
    Serial.print(frac,DEC) ;
  }
}
