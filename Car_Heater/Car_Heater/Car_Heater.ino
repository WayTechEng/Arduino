#include <DS3231.h>
#include <Wire.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
DS3231 myRTC;

const int pin_htr1 = 2;
const int pin_htr2 = 3;
const int pin_htr_man_enb = 11;
const int pin_acc_on = 12;
const int pin_backlight= 10;

int Pre_Sec;
bool backlight_override = false;

int htr2_state = 1; // 0=OFF, 1=ON

byte htr_start_hour = 6;
byte htr_start_min = 35;
byte htr_stop_min = 54;

byte init_hour = 17;
byte init_minute = 39;
byte init_second = 40;
byte init_dow = 5;

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

void debug_PrintHeaterManualEnableState(bool state)
{
    lcd.setCursor( 0, 1 );
    if(state == true)
    {      
      lcd.print("Man: H");
    }
    else
    {
      lcd.print("Man: L");
    }
}

void debug_PrintAccessoriesState(bool state)
{
    lcd.setCursor( 8, 1 );
    if(state == true)
    {      
      lcd.print("Acc: H");
    }
    else
    {
      lcd.print("Acc: L");
    }
}

void backlightControl(bool state, bool overrideValue)
{
  if(overrideValue)
  {
    digitalWrite(pin_backlight, HIGH);
  }
  else
  {
    digitalWrite(pin_backlight, state);
  }
}

void setup()
{
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Hello");
  lcd.setCursor( 0, 1 );
  delay(500);

  // I/O pins
  pinMode(pin_htr1, OUTPUT);
  pinMode(pin_htr2, OUTPUT);
  pinMode(pin_htr_man_enb, INPUT);
  pinMode(pin_acc_on, INPUT);
  pinMode(pin_backlight, OUTPUT);

  // clock
  // Serial.begin(9600);
  Wire.begin();
  ///// Enable this when setting the time initially
  // myRTC.setClockMode(false); // false = not using the alternate, 12-hour mode
  // myRTC.setHour(init_hour);
  // myRTC.setMinute(init_minute);
  // myRTC.setSecond(init_second);
  // myRTC.setDoW(init_dow);
}
    
void loop()
{
  bool h12;
  bool hPM;
  byte DoW = myRTC.getDoW();
  byte hour = myRTC.getHour(h12, hPM);
  byte minute = myRTC.getMinute();
  byte second = myRTC.getSecond();

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
    heaters(1,1);
    backlightControl(HIGH, backlight_override);
  }
  else // check the time
  {
    if( (DoW == 1) || (DoW == 3) || (DoW == 5) ) // This is mon, wed or fri
    {
      if( (hour == 6) && ( (minute >= htr_start_min) && (minute < htr_stop_min) ) )
      {
        heaters(1, htr2_state);
        backlightControl(HIGH, backlight_override);
      }
      else
      {
        heaters(0,0);
        backlightControl(LOW, backlight_override);
      }
    }
    else 
    {
      heaters(0,0);
      backlightControl(LOW, backlight_override);
    }
  } 
  
  printTimeToLCD(DoW, hour, minute, second);  
  debug_PrintHeaterManualEnableState(htr_man_state);
  debug_PrintAccessoriesState(acc_state);  

  delay(1000);
  int val = analogRead(A0);
  if(val >= 0 && 30 > val)
  {
    // Serial.print("RIGHT\n");
  }
  else if(val >= 30 && 150 > val)
  {
    // Serial.print("UP\n");
    byte new_minute = minute + 1;
    byte new_second = 0;    
    if(new_minute >= 60)
    {
      new_minute = 0;
      byte tHour = hour + 1; 
      myRTC.setHour(tHour);
    }
    myRTC.setMinute(new_minute);
    myRTC.setSecond(new_second);
  }
  else if(val >= 150 && 380 > val)
  {
    // Serial.print("DOWN\n");
    byte new_minute = minute - 1;
    byte new_second = 0;
    if((minute - 1) > minute)
    {
      new_minute = 60;
      byte tHour = hour - 1; 
      myRTC.setHour(tHour);
    }
    myRTC.setMinute(new_minute);
    myRTC.setSecond(new_second);
  }
  else if(val >= 380 && 580 > val)
  {
    // Serial.print("LEFT\n");
  }
  else if(val >= 580 && 990 > val)
  {
    // Serial.print("Select\n");
    backlight_override = !backlight_override;
  }

}
