#include <IRremote.h>

// Looking at sensor front on (looking at bulb).....Left prong....Middle prong..... Right prong
//                                                     Sig              GND              Vin

const int RECV_PIN = 12; // Signal in
IRrecv irrecv(RECV_PIN);
decode_results results;
String button;
bool high = false;

void setup() 
{
  Serial.begin(9600);
  irrecv.enableIRIn();
  irrecv.blink13(true);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

  if(high == true)
  {
    digitalWrite(pin, HIGH);
  }
  else
  {
    digitalWrite(pin, LOW);
  }

  if (irrecv.decode(&results))
  {
        Serial.println(results.value, HEX);
        delay(200);
        button = String(results.value, HEX);

        if (button == "ffe01f")     // vol-down
          {
            Serial.print("vol-down");
            Serial.print('\n');
            high = false;
          }
         if (button == "ffa857")    // vol-up
          {
            Serial.print("vol-up");
            Serial.print('\n');
            digitalWrite(pin, HIGH);
            high = true;
          }
        irrecv.resume();
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//                  Old motor code
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

/*
 * 

 Forward function
void motorForward()
{
  while (button != "ffc23d")
  {
    // Set up motor turn fowards
    digitalWrite(2,HIGH);
    digitalWrite(3,LOW);
    
    // Set motor speed
    analogWrite(9,speed);
    Serial.print("Speed Write");

    
    if (irrecv.decode(&results))
    {
       button = String(results.value, HEX);
       Serial.print(button);
       irrecv.resume();
    }
  }
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
}

// Reverse function
void motorReverse()
{
  while (button != "ffc23d")
  {
    // Set up motor turn fowards
    digitalWrite(2,LOW);
    digitalWrite(3,HIGH);
  
    // Set motor speed
    analogWrite(9,speed);

    if (irrecv.decode(&results))
    {
       button = String(results.value, HEX);
       Serial.print(button);
       irrecv.resume();
    }
  }
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
}


void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn();
  irrecv.blink13(true);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(9, OUTPUT);
}

void loop()
{
  if (irrecv.decode(&results))
  {
        Serial.println(results.value, HEX);
        Serial.print('\n');
        delay(100);
        button = String(results.value, HEX);
        Serial.print(button);
        if (button == "ffe01f")     // vol-down
          {
            Serial.print("vol-down");
            Serial.print('\n');
            Serial.print(speed);
            motorReverse();
          }
         if (button == "ffa857")    // vol-up
          {
            Serial.print("vol-up");
            Serial.print('\n');
            Serial.print(speed);
            motorForward();
          }
        irrecv.resume();
  }
}




// psuedo code


code does nothing unless button pressed.
The loop should only check for button presses.
each button press will (most likely) have its own function related to it.

motorForward - Press vol+ and the motor runs in forward direction
motorReverse - Press vol- and the motor runs in reverse direction




play/pause - ffc23d
*/
