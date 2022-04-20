#include "max6675.h" 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DS1307.h"

LiquidCrystal_I2C lcd(0x27, 2, 1, 0 , 4, 5, 6, 7, 3, POSITIVE);
int Pre_Sec;
DS1307 clock;//define a object of DS1307 class

// Global variables
double CALIBRATION_ADJUSTMENT = 2.5; 
int os_fan_speed = 150;
int is_fan_speed = 250;
float desired_temp = 20;
float tmp_set_point = 20;
float sum_temp = 0;
float hysterisis = 1.0;
float fridge_min = desired_temp - hysterisis; 
float fridge_max = desired_temp + hysterisis;
float average_temp = 0;
int loop_counter = 0;
int temp_get_counter = 0;
int number_of_iterations = 10;
int interations_get_temp = 10;
int temperature_edit_mode = 0;
int toggle_screen = 1;



// Pin declarations
int SO = 13;
int CS = 12;
int sck = 11;
int PWM_outside_fan = 10;
int PWM_inside_fan = 9;
int relay_fridge_en = 8;
int s_inc = 5;
int s_dec = 4;
int s_select = 3;
int s_next = 7;
int s_before = 6;

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

  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Hello");
  Serial.println("Hello");
}

void loop() 
{
  temp_get_counter++;
  if(temp_get_counter > interations_get_temp)
  {
    loop_counter++;    
    sum_temp += module.readCelsius();    
    temp_get_counter = 0;
  }
  
  if(loop_counter > number_of_iterations)
  {    
    average_temp = sum_temp/loop_counter - CALIBRATION_ADJUSTMENT;
    // Serial.println(average_temp);
    fridge_on_off(average_temp);
    display_information(average_temp);
    loop_counter = 0;
    sum_temp = 0;
  }

  int select_button = digitalRead(s_select);
  if(select_button == LOW)
  {
    delay(500);
    select_button = digitalRead(s_select);
    if(temperature_edit_mode == 1)  // if already editting temp, then this is the exit command.
    {
      if(select_button == LOW)
      {
        desired_temp = tmp_set_point;
        temperature_edit_mode = 0;
        Serial.println("Temperature edit mode DEACTIVATED..");
        display_information(average_temp);
        delay(1000);
        fridge_min = desired_temp - hysterisis; 
        fridge_max = desired_temp + hysterisis;
      }
    }
    else if(select_button == LOW)  // if not editting, then this is the command to begin editting temp.
    {
      Serial.println("Temperature edit mode activated..");
      temperature_edit_mode = 1;
      delay(1000);
    }    
  }

  if(temperature_edit_mode == 1)
  {
    int dec_button = digitalRead(s_dec);
    int inc_button = digitalRead(s_inc);
    if(dec_button == LOW)
    {
      tmp_set_point -= 0.5;
    }
    else if(inc_button == LOW)
    {
      tmp_set_point += 0.5;
    }

    // Screen flashing routien.
    if(toggle_screen == 1)
    {
      toggle_screen = 0;
      lcd.clear();
      // lcd.print("NEW TEMP: ");
      lcd.print(tmp_set_point);
      delay(200);
    }
    else
    {
      toggle_screen = 1;
      lcd.clear();
    }
    delay(200);
    
  }

  delay(100);
}

void fridge_on_off(float avg)
{
  if (fridge_max <= avg)
  {
    fridge_control(HIGH, os_fan_speed, is_fan_speed);
    Serial.println("Fridge = ON");
  }
  else if(avg <= fridge_min)
  {
    fridge_control(LOW, 0, 0);
    Serial.println("Fridge = OFF");
  }
  else
  {
    Serial.println("Fridge = NO CHANGE");
  }
}

void display_information(float avg)
{
  if(temperature_edit_mode == 0)
  {
    lcd.clear();
    lcd.print("SET TEMP: ");
    lcd.print(desired_temp);
    lcd.setCursor(0,1);
    lcd.print("TEMP: ");
    lcd.print(avg);
  }
}

// Controls the compressor state and fan states
void fridge_control(byte fridge_on, int outside_fan_pwm, int inside_fan_pwm)
{
  digitalWrite(relay_fridge_en, fridge_on);
  analogWrite(PWM_outside_fan, outside_fan_pwm);
  analogWrite(PWM_inside_fan, inside_fan_pwm);
}







  // button = digitalRead(s_dec);
  // if(button == LOW)
  // {
  //   Serial.println("DEC is pressed!");
  // }
  // button = digitalRead(s_inc);
  // if(button == LOW)
  // {
  //   Serial.println("INC is pressed!");
  // }
  // button = digitalRead(s_before);
  // if(button == LOW)
  // {
  //   Serial.println("BEFORE is pressed!");
  // }
  // button = digitalRead(s_select);
  // if(button == LOW)
  // {
  //   Serial.println("Select is pressed!");
  // }