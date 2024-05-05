// int an_pin = A3;
void setup() {
  // put your setup code here, to run once:
  pinMode(3, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // int value = analogRead(an_pin);
  int value = analogRead(3);
  analogWrite(3, value/4);
}
