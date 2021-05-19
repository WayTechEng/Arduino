const int buzzer = 0; //buzzer to arduino pin 9


void setup(){
 
  pinMode(buzzer, OUTPUT); // Set buzzer - pin 9 as an output

  pinMode(2, INPUT_PULLUP);

  pinMode(13, OUTPUT);
}

void loop(){
  int sensorVal = digitalRead(2);
  
  if (sensorVal == HIGH) {

    digitalWrite(13, LOW);

  } else {

    digitalWrite(13, HIGH);
    for (int i = 0; i < 5; i++)
    {
        tone(buzzer, 500); // Send 1KHz sound signal...
        delay(100);        // ...for 1 sec
        noTone(buzzer);     // Stop sound...
        delay(100);        // ...for 1sec
    }

  } 
}
