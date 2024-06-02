#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

void setup()
{
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Hello");
  lcd.setCursor( 0, 1 );
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  int val = analogRead(A0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(val);
  delay(200);
}
