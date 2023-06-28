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

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
// MPU6050 mpu(0x69); // <-- use for AD0 high

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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

float Kp = 60;
float Ki = 0;
float Kd = 0.00; // 0.07
float setpoint = 0.3;
double I_err = 0;
float prev_err = 0;

int off_thresh_L = -10;
int off_thresh_R = 10;
float angles[10] = {0,0,0,0,0,0,0,0,0,0};
int arr_size = 10;
int max_pwm = 255;

int en1 = 2;
int en2 = 4;
int out1 = 5;
int out2 = 6;
double dither_ratio = 350;
double dither_bounce = 1;
double dither_const = 0.05;

float dt_theory = 200;
float dt_actual = 6;
float accum = 0;
int counter = 0;
int n_counts = 20;
unsigned long t_s = micros();
unsigned long t_start = micros();

const int encoder_a = 7; // Pin 3
const int encoder_b = 8; // Pin 5
long encoder_pulse_counter = 0;
long direction = 1;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
void encoderPinChangeA()
{
    encoder_pulse_counter += 1;
    direction = digitalRead(encoder_a) == digitalRead(encoder_b) ? -1 : 1;
}
void encoderPinChangeB()
{
    encoder_pulse_counter += 1;
    direction = digitalRead(encoder_a) != digitalRead(encoder_b) ? -1 : 1;
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
    pinMode(en1, OUTPUT);
    pinMode(en2, OUTPUT);

    analogWrite(out1, 0);
    analogWrite(out2, 0);
    digitalWrite(en1, 1);
    digitalWrite(en2, 1);

    setPwmFrequency(pins_9_10, timer1_div); // timer 1

    pinMode(encoder_a, INPUT_PULLUP);
    pinMode(encoder_b, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(encoder_a), encoderPinChangeA, CHANGE);
    attachPCINT(digitalPinToPCINT(encoder_b), encoderPinChangeB, CHANGE);

    t_s = micros();
    t_start = micros();

    Serial.println("Program Begin!\n");
    
}


void loop()
{  
  // if( (micros() - t_start) >= dt_theory)
  // if( ((micros() - t_start) >= dt_theory) && (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)))
  int fifo_packet_ready = mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
  Serial.println(fifo_packet_ready);
  if(fifo_packet_ready)
  {
    Serial.println(fifo_packet_ready);
    t_start = micros();
    counter++;
    if(counter == n_counts)
    {
      dt_actual = (micros() - t_s) / n_counts;
      Serial.print("Time to complete 1 cycle: ");
      Serial.println(dt_actual);
      counter = 0;
      t_s = micros();
    }
  }  
}