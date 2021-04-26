void setup() 
{
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(9, OUTPUT);
}

void loop() 
{
  // Set up motor turn fowards
  digitalWrite(2,HIGH);
  digitalWrite(3,LOW);

  // Set motor speed
  analogWrite(9,150);
}
