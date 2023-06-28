// Constants
const int buttonPin = 2;     // the number encoder pin
const int mot1_1 = 2; // "Positive"
const int mot1_2 = 3; // "Negative"
const int en1 = 9; // Feed PWM signal to this pin

int PWM_val = 255*0.8; // 80% PWM
int count = 0;


void setup() {
  // Encoder
  pinMode(buttonPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), count_ISR, CHANGE);
  Serial.begin(9600);

  // Motor
  pinMode(mot1_1, OUTPUT);
  pinMode(mot1_2, OUTPUT);
  pinMode(en1, OUTPUT);
  
}

void loop() {
  int a = 1;
 /*
  analogWrite(en1, PWM_val); // between value between 0 and 255
  digitalWrite(mot_1, LOW);
  digitalWrite(mot_2, HIGH);

  delay(1000);

  digitalWrite(mot_1, LOW);
  digitalWrite(mot_2, LOW);
  analogWrite(en1, 0);

  delay(100000);*/
}

void count_ISR()
{
  count++;
  Serial.print(count);
  Serial.print("\n");
}
