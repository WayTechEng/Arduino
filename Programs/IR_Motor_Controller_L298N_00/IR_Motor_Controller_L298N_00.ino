/*
 * DOCUMENTATION
 * IR MOTOR CONTROLLER PROGRAM
 * 
 * Run this command for permissions
 * sudo chmod a+rw /dev/ttyACM0
 * 
 * M1 - Up/down motor
 * M2 - Tilt motor
 * 
 * mot1_1 - HIGH  |
 *                | = UP || DOWN
 * mot1_2 - LOW   |
 *-----------------
 * mot1_1 - LOW   |
 *                | = UP || DOWN
 * mot1_2 - HIGH  |    
 * ------------------------------------------
 * ------------------------------------------
 * mot2_1 - HIGH  |
 *                | = Extend
 * mot2_2 - LOW   |
 *-----------------
 * mot2_1 - LOW   |
 *                | = Retract
 * mot2_2 - HIGH  |    
 *     
*/
//////////////////////////////////////////////////////////////////////////////////////////
#include <IRremote.h>
const int RECV_PIN = 12; // Signal in
IRrecv irrecv(RECV_PIN);
decode_results results;
String button;
bool b_vol_up = false;
bool b_vol_down = false;
bool b_play = false;
bool b_prev = false;
const int led_pin = 10;
double ir_input_counter = 0;

// Constants
// M1
const int mot1_1 = 4; // always needs to be written high when actuating 
const int mot1_2 = 8; //
// M2
const int mot2_1 = 2; // "Positive"
const int mot2_2 = 3; // "Negative"
// PWM
const int enab_m2 = 9; // M2 Feed PWM signdelay(200);al to this pin
int PWM_m2 = 255; // 100% PWM

void setup() {
  // Motors
  pinMode(mot1_1, OUTPUT);
  pinMode(mot1_2, OUTPUT);
  pinMode(mot2_1, OUTPUT);
  pinMode(mot2_2, OUTPUT);
  pinMode(enab_m2, OUTPUT);
  pinMode(led_pin, OUTPUT);

  // IR sensor
  irrecv.enableIRIn();
  irrecv.blink13(true);

  // Serial
  Serial.begin(9600);
}

void loop() 
{
  if (irrecv.decode(&results))
  {
        delay(50); 
        Serial.println(results.value, HEX);
        // experiment with this value
          ir_input_counter = 0; // reset break-out counter
          
        if(results.value ==0xFFA857)
        {
          digitalWrite(mot1_1, HIGH);
          digitalWrite(mot1_2, HIGH);
          b_vol_up = true;
          Serial.println("vol-up");
        }
        else if(results.value == 0xFFE01F)
        {
          digitalWrite(mot1_1, HIGH);
          digitalWrite(mot1_2, LOW);
          b_vol_down = true;
          Serial.println("vol-down");
        }
        else if(results.value ==0XFFC23D)
        {
          digitalWrite(mot2_1, HIGH);
          digitalWrite(mot2_2, LOW);
          analogWrite(enab_m2, PWM_m2);
          b_play = true;
          Serial.println("Play");
        }
        else if(results.value ==0xFF22DD)
        {
          digitalWrite(mot2_1, LOW);
          digitalWrite(mot2_2, HIGH);
          analogWrite(enab_m2, PWM_m2);
          b_prev = true;
          Serial.println("Prev");
        }
        else if(results.value == 0xFFFFFFFF)
        {
          if(b_play)
          {
            b_Play();
          }
          else if(b_prev)
          {
            b_Prev();
          }
          else if(b_vol_up)
          {
            b_Vol_Up();
          }
          else if(b_vol_down)
          {
            b_Vol_Down();
          }
        }
        else
        {
          b_vol_up = false;
          b_vol_down = false;
          b_play = false;
          b_prev = false;
          digitalWrite(mot1_1, LOW);
          digitalWrite(mot1_2, LOW);
          digitalWrite(mot2_1, LOW);
          digitalWrite(mot2_2, LOW);
          analogWrite(enab_m2, 0);
          digitalWrite(led_pin, LOW);
          Serial.println("..............RESETTING EVERYTHING OFF..............");
        }

        irrecv.resume();
  }
  ir_input_counter++;
  //Serial.println(ir_input_counter);

  if(ir_input_counter >= 10000)
  {
    b_vol_up = false;
    b_vol_down = false;
    b_play = false;
    b_prev = false;
    digitalWrite(mot1_1, LOW);
    digitalWrite(mot1_2, LOW);
    digitalWrite(mot2_1, LOW);
    digitalWrite(mot2_2, LOW);
    analogWrite(enab_m2, 0);
    digitalWrite(led_pin, LOW);
    Serial.println("setting 0....");
    ir_input_counter = 0;
  }  
}

void b_Vol_Up()
{
  digitalWrite(mot1_1, HIGH);
  digitalWrite(mot1_2, HIGH);
  Serial.println("Function Vol_Up");
}

void b_Vol_Down()
{
  digitalWrite(mot1_1, HIGH);
  digitalWrite(mot1_2, LOW);
  Serial.println("Function Vol_Down");
}

void b_Play()
{
  digitalWrite(mot2_1, HIGH);
  digitalWrite(mot2_2, LOW);
  analogWrite(enab_m2, PWM_m2);
  Serial.println("Function b_Play");
}

void b_Prev()
{
  digitalWrite(mot2_1, LOW);
  digitalWrite(mot2_2, HIGH);
  analogWrite(enab_m2, PWM_m2);
  Serial.println("Function b_Prev");
}
