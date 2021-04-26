int led = 6;
int sound_digital = 7;
int sound_analog = A0;
int sum = 0;
int counter = 0;
int max_val = 0;

void setup(){
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  pinMode(sound_digital, INPUT);  
}

void loop(){
  int val_digital = digitalRead(sound_digital);
  int val_analog = analogRead(sound_analog);
  sum += val_analog;
  counter++;
  if(val_analog > max_val)
  {
    max_val = val_analog;
  }

  if(counter >= 1000)
  {
    int avg = sum/counter;
//    Serial.println(val_analog);
//    Serial.println(max_val);
    counter = 0;
    sum = 0;
    max_val = 0;
  }

  Serial.println(val_analog);
//  Serial.println("\t");
//  Serial.println(val_digital);
//
//  if (val_digital == HIGH)
//  {
//    digitalWrite (led, HIGH);
//    delay(3000);
//    }
//  else
//  {
//    digitalWrite (led, LOW);
//    }
}
