int AC_LOAD = 3;
int dimming = 128;

void setup() 
{
  // put your setup code here, to run once:
  pinMode(AC_LOAD, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), zero_cross_int, RISING);
}

void zero_cross_int()
{
  int dimTime = (75*dimming);
  delayMicroseconds(dimTime);

  digitalWrite(AC_LOAD, HIGH);
  delayMicroseconds(10);

  digitalWrite(AC_LOAD, LOW);
  
}

void loop() 
{
    dimming = 125;
    delay(10);
  
}
