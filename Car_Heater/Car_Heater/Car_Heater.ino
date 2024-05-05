#include <DS3231.h>
#include <Wire.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

const int pin_htr1 = 2;
const int pin_htr2 = 3;
const int pin_htr_man_enb = 11;
const int pin_acc_on = 12;

DS3231 myRTC;

int Pre_Sec;

int htr2_state = 1; // 0=OFF, 1=ON

byte htr_start_hour = 6;
byte htr_start_min = 30;
byte htr_stop_min = 54;

byte init_hour = 16;
byte init_minute = 10;
byte init_second = 55;

void printDayOfWeek(byte theWeekDay)
{
  switch (theWeekDay)
  {
    case 1:
      lcd.print("Mon");
      break;
    case 2:
      lcd.print("Tue");
      break;
    case 3:
      lcd.print("Wed");
      break;
    case 4:
      lcd.print("Thu");
      break;
    case 5:
      lcd.print("Fri");
      break;
    case 6:
      lcd.print("Sat");
      break;
    case 7:
      lcd.print("Sun");
      break;
  }
}

void printTimeToLCD(byte DoW, byte hour, byte minute, byte second)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(hour);
  lcd.print(":");
  lcd.print(minute);
  lcd.print(":");
  lcd.print(second);
  lcd.print(", ");
  printDayOfWeek(DoW);
}

void heaters(int htr1_on, int htr2_on)
{
  if(htr1_on)
  {
    digitalWrite(pin_htr1, HIGH);
  }
  else
  {
    digitalWrite(pin_htr1, LOW);
  }

  if(htr2_on)
  {
    digitalWrite(pin_htr2, HIGH);
  }
  else
  {
    digitalWrite(pin_htr2, LOW);
  }
}

void setup()
{
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Hello");
  lcd.setCursor( 0, 1 );
  delay(2000);

  // I/O pins
  pinMode(pin_htr1, OUTPUT);
  pinMode(pin_htr2, OUTPUT);
  pinMode(pin_htr_man_enb, INPUT);
  pinMode(pin_acc_on, INPUT);

  // clock
  //  Serial.begin(9600);
  Wire.begin();
  myRTC.setClockMode(false); // false = not using the alternate, 12-hour mode
  myRTC.setHour(init_hour);
  myRTC.setMinute(init_minute);

}
    
void loop()
{
  bool h12;
  bool hPM;
  byte DoW = myRTC.getDoW();
  byte hour = myRTC.getHour(h12, hPM);
  byte minute = myRTC.getMinute();
  byte second = myRTC.getSecond();

  printTimeToLCD(DoW, hour, minute, second);
  delay(1000);

  bool htr_man_state = digitalRead(pin_htr_man_enb);
  bool acc_state = digitalRead(pin_acc_on);
  if(acc_state == HIGH)
  {
    htr2_state = 0;
  }
  else
  {
    htr2_state = 1;
  }

  if(htr_man_state == HIGH) // force the seat heater on only
  {
    heaters(1,0);
  }
  else // cheap the time
  {
    if( (DoW == 1) || (DoW == 3) || (DoW == 5) ) // This is mon, wed or fri
    {
      if( (hour == 6) && ( (minute >= htr_start_min) && (minute <= htr_stop_min) ) )
      {
        heaters(1, htr2_state);
      }
      else
      {
        heaters(0,0);
      }
    }
    else 
    {
      heaters(0,0);
    }
  } 
  


}
