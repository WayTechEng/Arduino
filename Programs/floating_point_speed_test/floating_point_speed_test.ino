float A = 10.0;
float B = 20.0;
int a = 1;
int b = 20;
long c = 20;
float storage_float = 0.0;
unsigned long storage_int = 0;
long storage_long = 0;


unsigned long t_start = micros();
int counter = 0;
int n_times = 10000;

void setup() 
{
  Serial.begin(115200);

}

void loop() 
{
  for(long i=1; i < 2; i++)
  {
    // storage_float += A*B;
    // storage_int += c*a/i;
    // if(Serial.available() > 0)
    // {
    //   int a = Serial.read();
    // }
    storage_long += (10 * 19999)/20000;
  }  
  // storage_int = a*b
  // storage_int = a*b
  // 
  counter++;
  if(counter == n_times)
  {    
    Serial.print("\nTime (us): ");
    unsigned long time_per_calc = (micros() - t_start)/10;
    // unsigned long time_per_calc = (micros() - t_start)/n_times;
    Serial.println(time_per_calc);
    Serial.println(storage_long);
    // Serial.println(storage_float);
    counter = 0;
    t_start = micros();
  }  
}
