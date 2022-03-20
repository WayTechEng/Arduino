#include "max6675.h" 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DS1307.h"

LiquidCrystal_I2C lcd(0x27, 2, 1, 0 , 4, 5, 6, 7, 3, POSITIVE);
int Pre_Sec;
DS1307 clock;//define a object of DS1307 class

// Global variables
int os_fan_speed = 150;
int is_fan_speed = 250;
float desired_temp = 30;
float sum_temp = 0;
float average_temp =0;
float hysterisis = 1.5;
float fridge_min = desired_temp - hysterisis; 
float fridge_max = desired_temp + hysterisis; 
int loop_counter = 0;
int number_of_iterations = 5;


// Pin declarations
int SO = 13;
int CS = 12;
int sck = 11;
int PWM_outside_fan = 10;
int PWM_inside_fan = 9;
int relay_fridge_en = 8;
int s_inc = 7;
int s_dec = 6;
int s_select = 5;
int s_next = 4;
int s_before = 3;

// Initial thermocouple module
MAX6675 module(sck, CS, SO);

void setup() {
  // Pin modes
  pinMode(PWM_outside_fan, OUTPUT);
  pinMode(PWM_inside_fan, OUTPUT);
  pinMode(relay_fridge_en, OUTPUT);
  pinMode(s_inc, INPUT_PULLUP);
  pinMode(s_dec, INPUT_PULLUP);
  pinMode(s_select, INPUT_PULLUP);
  pinMode(s_next, INPUT_PULLUP);
  pinMode(s_before, INPUT_PULLUP);
  digitalWrite(relay_fridge_en, LOW);
  analogWrite(PWM_outside_fan, 0);
  analogWrite(PWM_inside_fan, 0);

  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Hello");
}

void loop() {
  loop_counter++;
  sum_temp += module.readCelsius();
  if(loop_counter > number_of_iterations)
  {
    average_temp = sum_temp/loop_counter; 

    if (fridge_max <= average_temp)
    {
      fridge_control(HIGH, os_fan_speed, is_fan_speed);
      Serial.println("Fridge = ON");
    }
    else if(average_temp <= fridge_min)
    {
      fridge_control(LOW, 0, 0);
      Serial.println("Fridge = OFF");
    }   
    
    lcd.clear();
    display_information();

    loop_counter = 0;
    sum_temp = 0;
  }
  delay(1000);
}

// Controls the compressor state and fan states

void display_information()
{
  lcd.print("Set temp: ");
  lcd.print(desired_temp);
  lcd.setCursor(0,1);
  lcd.print("temp: ");
  lcd.print(average_temp);
}

void fridge_control(byte fridge_on, int outside_fan_pwm, int inside_fan_pwm)
{
  digitalWrite(relay_fridge_en, fridge_on);
  analogWrite(PWM_outside_fan, outside_fan_pwm);
  analogWrite(PWM_inside_fan, inside_fan_pwm);
}
