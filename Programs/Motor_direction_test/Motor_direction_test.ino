#include <Arduino.h>

// Pins
int M1_extend_pin = 6;
int M1_retract_pin = 9;
int M2_extend_pin = 3;
int M2_retract_pin = 5;
int IR_pin = 2;
int fan_pin = 10;
int sw_top_extend = 7;
int sw_top_retract = 8;
int sw_bot_extend = 12;
int sw_bot_retract = 4;
int sw_door = 11;

// PWMs
int M1_extend_PWM = 0;
int M1_retract_PWM = 0;
int M2_extend_PWM = 0;
int M2_retract_PWM = 0;
int fan_PWM = 0;



// global variables
bool retracting = false;
bool extending = false;

float time_till_full_speed = 4000;
unsigned long start_timer;
unsigned long reversal_timer;
int reversal_lag_duration = 2000;
bool instant_reversal_avoided = false;
  bool avoid_instant_reversal = false;

int M1_max_PWM = 50;
int M2_max_PWM = 200;

bool move_once = true;

void setup() {
  Serial.begin(9600);
  Serial.println("Initialising..\n");

  // Outputs
  pinMode(M1_extend_pin, OUTPUT);
  pinMode(M1_retract_pin, OUTPUT);
  pinMode(M2_extend_pin, OUTPUT);
  pinMode(M2_retract_pin, OUTPUT);
  pinMode(fan_pin, OUTPUT);
  // Inputs
  pinMode(sw_top_extend, INPUT_PULLUP); // high if switch activated
  pinMode(sw_top_retract, INPUT_PULLUP);
  pinMode(sw_bot_extend, INPUT_PULLUP);
  pinMode(sw_bot_retract, INPUT_PULLUP);
  pinMode(sw_door, INPUT_PULLUP);

  // Initialising outputs
  analogWrite(M1_extend_pin, 0);
  analogWrite(M1_retract_pin, 0);
  analogWrite(M2_extend_pin, 0);
  analogWrite(M2_retract_pin, 0);
  analogWrite(fan_pin, 0);

}

void loop() {
  if(move_once)
  {
    move_once = false;
    M1_extend_PWM = M1_max_PWM;
    
  }
  else
  {
    M1_extend_PWM = 0;
    M1_retract_PWM = 0;
    M2_extend_PWM = 0;
    M2_retract_PWM = 0;
    fan_PWM = 0;
  }
  analogWrite(M1_extend_pin, M1_extend_PWM);
  analogWrite(M1_retract_pin, M1_retract_PWM);
  analogWrite(M2_extend_pin, M2_extend_PWM);
  analogWrite(M2_retract_pin, M2_retract_PWM);
  analogWrite(fan_pin, fan_PWM);
  delay(500);

}