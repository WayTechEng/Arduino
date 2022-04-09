#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DS1307.h"

int charging_led = 7;
int charging_enable = 4;
int bat_voltage_pin = A0;
int raw_val = 0;
double max_voltage = 12.8;
unsigned long start_time = millis();
int loop_counter = 0;
int num_iterations = 5;
double voltage_sum = 0;
int charging_interval = 20;
unsigned long minutes_to_charge = 180L;

LiquidCrystal_I2C lcd(0x27, 2, 1, 0 , 4, 5, 6, 7, 3, POSITIVE);

void setup() {
  // put your setup code here, to run once:
  pinMode(charging_led, OUTPUT);
  pinMode(charging_enable, OUTPUT);

  Serial.begin(9600);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Hello");
  digitalWrite(charging_led, LOW);
  digitalWrite(charging_enable, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  raw_val = analogRead(bat_voltage_pin);
  double bat_voltage = max_voltage * (raw_val/1023.0);
  voltage_sum += bat_voltage;
  loop_counter++;

  if (loop_counter > num_iterations)
  {
    avg_voltage = voltage_sum / loop_counter;
    
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("B_VOLTS: ");
    lcd.print(avg_voltage);
    
    if(avg_voltage < min_bat_thresh)
    {
      digitalWrite(charging_enable, HIGH);
      digitalWrite(charging_led, HIGH);
      lcd.clear();
      lcd.print("MinVoltDetect");
      lcd.setCursor(0,1);
      lcd.print("CHARGING...");
      
      unsigned long milli_secs = minutes_to_charge * 60 * 1000;
      delay(milli_secs);
      digitalWrite(charging_enable, LOW);
      digitalWrite(charging_led, LOW);
    }

    unsigned long elapsed_time = start_time - millis();
    //                                    seconds    minutes   hours
    unsigned long hours = elapsed_time /   1000   /   60    /   60

    if(hours >= charging_interval)
    {
      lcd.clear();
      lcd.print("Planned");
      lcd.setCursor(0,1);
      lcd.print("CHARGING");
      digitalWrite(charging_enable, HIGH);
      digitalWrite(charging_led, HIGH);

      unsigned long milli_secs = minutes_to_charge * 60 * 1000;
      delay(milli_secs);
      digitalWrite(charging_enable, LOW);
      digitalWrite(charging_led, LOW);
      start_time = millis();
    }

    loop_counter = 0;
    voltage_sum = 0;
  }
  

  Serial.println(raw_val);
//  printDouble(test, 3);
//  Serial.println(".. .. ");
//  Serial.println("");
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
