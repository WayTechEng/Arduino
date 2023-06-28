

void setup() {
  // put your setup code here, to run once:
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  analogWrite(3, 120);
  analogWrite(5, 120);
  analogWrite(10, 120);
  analogWrite(11, 120);
  
}
