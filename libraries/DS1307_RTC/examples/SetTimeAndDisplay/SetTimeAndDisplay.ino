#include <Wire.h>
#include "DS1307.h"

int Pre_Sec;
DS1307 clock;//define a object of DS1307 class
void setup()
{
	Serial.begin(9600);
	clock.begin();
	clock.fillByYMD(2017,8,18);//AUG 18,2017
	clock.fillByHMS(12,00,00);//12:00:00"
	clock.fillDayOfWeek(SAT);//Saturday
	clock.setTime();//write time to the RTC chip
  Pre_Sec=clock.second;
}
void loop()
{
	printTime();
}
/*Function: Display time on the serial monitor*/
void printTime()
{
	clock.getTime();
	if(clock.second!=Pre_Sec)
	{
      Serial.print(clock.hour, DEC);
    	Serial.print(":");
    	Serial.print(clock.minute, DEC);
    	Serial.print(":");
    	Serial.print(clock.second, DEC);
    	Serial.print("	");
    	Serial.print(clock.month, DEC);
    	Serial.print("/");
    	Serial.print(clock.dayOfMonth, DEC);
    	Serial.print("/");
    	Serial.print(clock.year+2000, DEC);
    	Serial.print(" ");
    	switch (clock.dayOfWeek)// Friendly printout the weekday
    	{
    		case MON:
    		  Serial.print("MON");
    		  break;
    		case TUE:
    		  Serial.print("TUE");
    		  break;
    		case WED:
    		  Serial.print("WED");
    		  break;
    		case THU:
    		  Serial.print("THU");
    		  break;
    		case FRI:
    		  Serial.print("FRI");
    		  break;
    		case SAT:
    		  Serial.print("SAT");
    		  break;
    		case SUN:
    		  Serial.print("SUN");
    		  break;
    	}
    	Serial.println(" ");
		Pre_Sec=clock.second;
	}
}