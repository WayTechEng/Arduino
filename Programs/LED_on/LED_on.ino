  const int led_pin = 13;
  double a = 0;
void setup() {
  // put your setup code here, to run once:
  
  pinMode(led_pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  a++;

  while( (a>1000) && (a<2000) )
  {
    digitalWrite(led_pin, HIGH);
  }

}
