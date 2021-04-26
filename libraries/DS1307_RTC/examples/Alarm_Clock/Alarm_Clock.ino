#include <Wire.h>
#include "DS1307.h"

DS1307 clock;//define a object of DS1307 class
void setup()
{
	clock.begin();
	clock.fillByYMD(2017,8,18);//AUG 18,2017
	clock.fillByHMS(12,00,00);//12:00:00"
	clock.fillDayOfWeek(SAT);//Saturday
	clock.setTime();//write time to the RTC chip
  pinMode(2,OUTPUT);
}
void loop()
{
	clock.getTime();
	if(clock.second==0)
	digitalWrite(2,HIGH);
	if(clock.second==10)
	digitalWrite(2,LOW);
}
/*Function: Display time on the serial monitor*/
