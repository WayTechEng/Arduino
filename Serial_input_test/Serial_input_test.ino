unsigned long t_start = millis();
double counts = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello");
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0)
  {
    int a = 0;
    a = Serial.parseInt();
    if(a != 0)
    {
      Serial.print("Inputted text is: ");
      Serial.println(a);
    }
  }

  counts++;
  if(counts > 50000)
  {
    double st = 1000*(millis() - t_start)/50000;
    Serial.print("Time: ");
    Serial.println(st);
    t_start = millis();
    counts = 0;
  }
    
}
