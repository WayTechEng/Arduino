#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0 ,4, 5, 6, 7, 3, POSITIVE);
const int buttonPin = 2;
int buttonState = 0;
bool trig = false;
bool resetTime = false;

void setup() {
  lcd.begin(16,2);
  lcd.clear();
  lcd.print("Hello");
  lcd.setCursor( 0,1 );
  delay(5000);
  lcd.clear();
  lcd.print("REX HAS NOT BEEN");
  lcd.setCursor( 0,1 );
  lcd.print("FED");
  pinMode(buttonPin, INPUT);  
}

void loop() 
{
  buttonState = digitalRead(buttonPin);

  if((buttonState == HIGH) && (trig == false))
  {
    lcd.clear();
    lcd.print("REX HAS BEEN FED");
    trig = true;
  }

  if(resetTime == true)
  {
    lcd.clear();
    lcd.print("REX HAS NOT BEEN");
    lcd.setCursor( 0,1 );
    lcd.print("FED");
    trig = false;
    resetTime == false;
    delay(10000);
  }

  
}
