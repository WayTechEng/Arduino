const int mot1 = 2; // "Positive"
const int mot2 = 3; // "Negative"
const int en1 = 9; // Feed PWM signal to this pin
int PWM_val = 255*0.8; // 80% PWM


void setup() 
{
  pinMode(mot1, OUTPUT);
  pinMode(mot2, OUTPUT);
  pinMode(en1, OUTPUT);
}

void loop() 
{
  analogWrite(en1, PWM_val); // between value between 0 and 255
  digitalWrite(mot1, LOW);
  digitalWrite(mot2, HIGH);

  delay(2000);
  analogWrite(en1, 0); // between value between 0 and 255
  digitalWrite(mot1, LOW);
  digitalWrite(mot2, LOW);
  delay(10000);

}

// Use IR sensor to change motors.
// start m1 dir 1 button
// stop m1 button
// start m1 dir 2 button
// start m2 dir 1 button
// stop m2 button
// start m2 dir 2 button
