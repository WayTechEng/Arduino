const int pwm_pin = 10;
const int rx_pin = 0;
const int pin_read = 4;

void setup() {
  pinMode(pwm_pin, OUTPUT);
  pinMode(pin_read, INPUT);
//  Serial.begin(115200);
  Serial.begin(1000000);
}

void loop() {
  analogWrite(pwm_pin, 155);
  int val = digitalRead(pin_read);
//  float i = 100.0f*analogRead(A0)/1024.0f;
  Serial.println(val);
}
