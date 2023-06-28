volatile int count = 0;
long int i = 0;
int cassy = 10;
int n;

void setup()
{
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), counter, HIGH);
}

void counter()
{
  count++;
}

void loop()
{
  delay(100);
  Serial.print(count);
  Serial.print("\n");
}

