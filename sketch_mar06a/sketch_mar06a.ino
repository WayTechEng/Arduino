volatile int count = 0;
long int i = 0;
int cassy = 10;
int n;

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(2), counter, RISING);
  Serial.begin(9600);
}

void counter()
{
  count++;
}
void loop() {
  // put your main code here, to run repeatedly:
  
  Serial.print(count);
  Serial.print("\n");
}
