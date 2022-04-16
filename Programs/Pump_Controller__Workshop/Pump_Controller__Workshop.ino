#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DS1307.h"

int charging_led = 7;
int charging_enable = 4;
int bat_voltage_pin = A0;
int raw_val = 0;
double max_voltage = 12.8;
unsigned long start_time = 0;
int loop_counter = 0;
int num_iterations = 5;
double voltage_sum = 0;
int waiting_interval = 20;  // Hours * minutes * seconds  == time in seconds
long milli_secs_to_charge = 7200000;
int min_bat_thresh = 12;

LiquidCrystal_I2C lcd(0x27, 2, 1, 0 , 4, 5, 6, 7, 3, POSITIVE);

void setup() {
  // put your setup code here, to run once:
  pinMode(charging_led, OUTPUT);
  pinMode(charging_enable, OUTPUT);

  Serial.begin(9600);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Initialising");
  lcd.setCursor(0,1);
  lcd.print("..3");
  delay(1000);
  lcd.print("..2");
  delay(1000);
  lcd.print("..1");
  digitalWrite(charging_led, LOW);
  digitalWrite(charging_enable, LOW);

  start_time = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  raw_val = analogRead(bat_voltage_pin);
  double bat_voltage = max_voltage * (raw_val/1023.0);
  voltage_sum += bat_voltage;
  loop_counter++;
  
  // Serial.println(raw_val);
  // printDouble(bat_voltage, 3);
  // Serial.println("");
  // lcd.clear();
  // lcd.setCursor(0,1);
  // lcd.print("B_VOLTS: ");
  // lcd.print(bat_voltage);
  
  if (loop_counter > num_iterations)
  {
    unsigned long elapsed_time = millis() - start_time;
    //                                    seconds    minutes
    // unsigned long minutes = elapsed_time /   1000   /   60;
    double minutes = double(elapsed_time) /   1000.0   /   60.0    /    60.0;

    double avg_voltage = voltage_sum / loop_counter;
    
    // lcd.clear();
    // lcd.setCursor(0,1);
    // lcd.print("B_VOLTS: ");
    // lcd.print(avg_voltage);
    
    if(avg_voltage < min_bat_thresh)
    {
      // Serial.println("Charging - Min voltage detected");
      digitalWrite(charging_enable, HIGH);
      digitalWrite(charging_led, HIGH);
      lcd.clear();
      lcd.print("MinVoltDetect");
      lcd.setCursor(0,1);
      lcd.print("CHARGING...");
      
      delay(milli_secs_to_charge);
      digitalWrite(charging_enable, LOW);
      digitalWrite(charging_led, LOW);
    }
    else if(minutes >= waiting_interval)
    {
      // Serial.println("Charging - periodic charging activated");
      lcd.clear();
      lcd.print("PLANNED");
      lcd.setCursor(0,1);
      lcd.print("CHARGING:");
      lcd.print(milli_secs_to_charge/1000/60);   

//      lcd.print(hours);
//      lcd.setCursor(0,1);
//      lcd.print(elapsed_time);
      
      digitalWrite(charging_enable, HIGH);
      digitalWrite(charging_led, HIGH);      
      delay(milli_secs_to_charge);
      digitalWrite(charging_enable, LOW);
      digitalWrite(charging_led, LOW);
      start_time = millis();
    }
    else
    {
      lcd.clear();
      lcd.print("WAITING");
      lcd.setCursor(0,1);
      lcd.print(minutes);
      lcd.print("/");
      lcd.print(waiting_interval);
      lcd.print(" hrs");
    }    

    // delay(1000);
    
    loop_counter = 0;
    voltage_sum = 0;
    // Serial.println("Resetting loop counter\n-------------------------------------------\n");
  }
  

  
//  
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
