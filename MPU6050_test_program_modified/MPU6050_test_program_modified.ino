// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include <PWMFreak.h>
// #include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "PinChangeInterrupt.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/*
    wheel_rev/min = (pls/min) / ( pls/wheel_rev )
                                  ^
                              Constant

    Encoder Pulse = pls
    pls/wheel_rev = 1008
    PLS/SEC = (1000msec/sec)* PLS/mSEC 
    PLS/MIN = (60sec/min) * PLS/SEC = (60000msec/min) * PLS/mSEC
    
    wheel_rev/min = 60000 * pls/msec

    // <---- (60000*encoder_pulse_counter/(dt_theory_ms)) /1008;

    // <---- 60000/3*encoder_pulse_counter_sum/(dt_actual_us);

*/

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
// MPU6050 mpu(0x69); // <-- use for AD0 high

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// Pin declarations
int en = 4;
int out1 = 5;
int out2 = 6;
const int encoder_a = 7; // Pin 3
const int encoder_b = 8; // Pin 5
// https://nerdytechy.com/how-to-change-the-pwm-frequency-of-arduino/
// https://www.etechnophiles.com/change-frequency-pwm-pins-arduino-uno/
int pins_9_10 = 9;  // timer 1
int timer1_div = 1;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float Kp_ang = 10;
float Ki_ang = 0.0;
float Kd_ang = 0.00;
float Kp_rpm = 0.73;
float Ki_rpm = 0.1;
float Kd_rpm = 0.0;
float angle_sp = 0.3;
int rpm_sp = 0;
float I_err_angle = 0;
float prev_err_angle = 0;
int I_err_rpm = 0;
int prev_err_rpm = 0;
int max_pwm = 255;
int max_rpm = 300;

int off_thresh_L = -10;
int off_thresh_R = 10;
int dither_bounce = 1;
float dither_ratio = 350;
float dither_const = 0.05;
float accum = 0;
int counter = 0;
int n_counts = 200;
signed long encoder_pulse_counter = 0;
int desired_direction = 1;
int cur_direction = 1;
int pulses_arr[3] = {0,0,0};
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
bool fifo_avaiable = false;
bool check_for_fifo = true;

long dt_theory_us = 10000; // 10000us = 10ms
long dt_theory_ms = dt_theory_us/1000;
long dt_actual_us = 10000;
long dt_actual_ms = dt_actual_us/1000;
unsigned long t_start_actual = micros();
unsigned long t_start_loop = micros();

void dmpDataReady() {
    mpuInterrupt = true;
}
void encoderPinChangeA()
{
    encoder_pulse_counter += 1;
    cur_direction = digitalRead(encoder_a) == digitalRead(encoder_b) ? -1 : 1;
}
void encoderPinChangeB()
{
    encoder_pulse_counter += 1;
    cur_direction = digitalRead(encoder_a) != digitalRead(encoder_b) ? -1 : 1;
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        // Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    if (!dmpReady)
    {
      Serial.println("Failed to init. Please reboot...");
      while(true){}
    }

    // configure pins
    pinMode(LED_PIN, OUTPUT);

    pinMode(out1, OUTPUT);
    pinMode(out2, OUTPUT);
    pinMode(en, OUTPUT);

    analogWrite(out1, 0);
    analogWrite(out2, 0);
    digitalWrite(en, 1);

    setPwmFrequency(pins_9_10, timer1_div); // timer 1

    pinMode(encoder_a, INPUT_PULLUP);
    pinMode(encoder_b, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(encoder_a), encoderPinChangeA, CHANGE);
    attachPCINT(digitalPinToPCINT(encoder_b), encoderPinChangeB, CHANGE);
}

void loop()
{  
  int pwm = 0;

  // bool result = (micros() - t_start_loop) >= dt_theory_us;
  if(check_for_fifo)
  {
    if(mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
      fifo_avaiable = true;
      check_for_fifo = false;
    }
  }
  // if( ((micros() - t_start_loop) >= dt_theory_us) && (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)))
  // if( result && (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)))
  // if(fifo_avaiable && result)
  if(fifo_avaiable && ((micros() - t_start_loop) >= dt_theory_us))
  // if( (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)))
  // if(false)
  {
    fifo_avaiable = false;
    check_for_fifo = true;
    // Serial.println("\n--------------------------");
    // Serial.print("\nMicros now: ");
    // Serial.print(micros());
    // Serial.print("\nT Start: ");
    // Serial.print(t_start_loop);

    // Serial.print("\nCalc: ");
    // Serial.print(micros());
    // Serial.print(" - ");
    // Serial.print(t_start_loop);
    // Serial.print("\nAnswer: ");
    // Serial.print(micros() - t_start_loop);

    t_start_loop = micros();   
    ///////////////////////////////////////////////////////////
    // Code adding tests
    int encoder_pulse_counter_sum = 0;
    // int rpm = 0;
    int rpm_avg = 0;
    int rpm_error = 0;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);    
    double cur_angle = ypr[1]*180/M_PI-.8; 

    for(int i=2;i>0;i--) // add values to rolling pulses-moving-average
    {
      pulses_arr[i] = pulses_arr[i-1];
      encoder_pulse_counter_sum += pulses_arr[i-1];
    }
    pulses_arr[0] = encoder_pulse_counter;
    encoder_pulse_counter_sum += encoder_pulse_counter;    

    
    // rpm = 60000*encoder_pulse_counter/(dt_actual_us); // <---- (60000*encoder_pulse_counter/(dt_theory_ms)) /1008;
    rpm_avg = ((cur_direction*20000)*encoder_pulse_counter_sum)/dt_actual_us;  // <---- cur_direction*60000/3*encoder_pulse_counter_sum/(dt_actual_us);
    encoder_pulse_counter = 0;

    Serial.println("Current angle: ");
    Serial.println(cur_angle);



    //////////////////////////////////////////////////////////
    // Time keeping
    counter++;
    if(counter == (n_counts))
    {
      // Serial.println(t_start_actual);
      // Serial.println(micros());
      dt_actual_us = (micros() - t_start_actual) / n_counts;
      dt_actual_ms = dt_actual_us/1000;
      Serial.print("\nTime to complete 1 cycle: ");
      Serial.println(dt_actual_us);
      counter = 0;
      t_start_actual = micros();
    }
    
  }  
}

