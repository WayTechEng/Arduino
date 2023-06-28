#include "max6675.h" 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DS1307.h"

LiquidCrystal_I2C lcd(0x27, 2, 1, 0 , 4, 5, 6, 7, 3, POSITIVE);
int Pre_Sec;
DS1307 clock;//define a object of DS1307 class

int SO = 13;
int CS = 12;
int sck = 11;
MAX6675 module(sck, CS, SO);

void setup() {   
  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Hello");
  lcd.setCursor( 0, 1 );
}

void loop() {
  float temperature = module.readCelsius();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(F("°C "));
  lcd.clear();
  lcd.print("T: ");
  lcd.print(temperature);
  delay(1000);  
}
