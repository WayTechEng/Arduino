// constants won't change. They're used here to set pin numbers:
const int buttonPin = 3;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int count = 0;

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(3), count_ISR, CHANGE);
  Serial.begin(9600);
}

void loop() {
  // read the state of the pushbutton value:
  //buttonState = digitalRead(buttonPin);
  /*
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == LOW) {
  } 
  else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }*/
}


void count_ISR()
{
  digitalWrite(ledPin, HIGH);
  count++;
  Serial.print(count);
  Serial.print("\n");
}


