/* Import External Packages */
#include <NewPing.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/* Variable Definitions */
  // #define 
    #define baudRate 9600
  
    #define E1Pin 10 //PWM-1
    #define E2Pin 11 //PWM-2
    #define M1Pin 12 //DIR-1
    #define M2Pin 13 //DIR-2
    
    #define forward LOW
    #define reverse HIGH
    #define left 0
    #define right 1

    #define speed1 20

    #define triggerPin 7
    #define echoPin 6
    #define minWallDist 10
    #define range 3000 // the maximum distance allowed for the ultrasonic sensor

    #define interruptPin 2

    #define mineSensitivity 400

  // boolean 
    bool runYet = false;
  
  // integer
    int magx = 0;
    int magy = 0;
    int magz = 0;
    int minesDetected = 0;

    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[128]; // FIFO storage buffer
    volatile bool mpuInterrupt = false;
    
    MPU6050 mpu;
    
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    //float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    
    unsigned long initTime = 0;
    unsigned long debounceDelay = 1000;
    int l_speed, r_speed;
    int initial_heading;
    bool start = false; // status of run
    bool runyet = false; // status of runoncebool dmpReady = false;  // set true if DMP init was successful
    
    // states for state machine
    #define stateDetectMine 0
    #define stateMarkMine 10
    #define stateDetectWall 20
    #define stateStartTurn 21
    #define stateTurnAround 30
    #define stateBreak 40
    #define stateStop 50
    
    // variables for state machine layer 1
    int initialHeading;
    int projectedHeading;
    int turnDirection;
    
    #define clockwise 1
    #define counterclockwise 2
    #define neutral 3
    
    char state = "detect mine";

    #define startbtn A3

/* Creation of Objects */
  // New Ping Library
    NewPing sonar(triggerPin, echoPin, range);
  // Adafruit HMC8553 Library
    Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void init_motors(){
    pinMode(E1Pin, OUTPUT);
    pinMode(M1Pin, OUTPUT);
    pinMode(E2Pin, OUTPUT);
    pinMode(M2Pin, OUTPUT);
}

void init_ultrasonic(){
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void init_magnetometer(){
  if(!mag.begin()){
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

//  displaySensorDetails();
  Serial.println("HMC5883 Magnetometer Initialised");
}

void init_gyro(){
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

  while (!Serial);
//  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(interruptPin, INPUT);
//  Serial.println(F("Testing device connections..."));
//  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

//  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {
      mpu.setDMPEnabled(true);

//      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      attachInterrupt(digitalPinToInterrupt(interruptPin), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
//      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
}

void init_components(){
  pinMode(startbtn, INPUT);
  digitalWrite(startbtn, HIGH);
}

void setup(){
  Serial.begin(baudRate);
  init_motors();
  Serial.println("Motors Initialised");
  init_ultrasonic();
  Serial.println("Ultrasonic Initialised");
  init_magnetometer();
  Serial.println("Magnetometer Initialised");
  init_gyro();
  Serial.println("Gyro Initialised");
  init_components();
  Serial.println("Components Initialised");

  Serial.println("Waiting for Start");
  wait_for_start(); // clear fifo while in block operation
  
  steer(40, 40);
  Serial.println("Starting motors");

  turnDirection = clockwise;

  state = stateDetectMine; // start the switch case at the top of the stack
}

void loop(){
  switch (state){
    case stateDetectMine:
//      Serial.println("stateDetectMine");
      if (detectMine()){
//        Serial.println("Mine detected!");
        state = stateMarkMine;
      }
      else{
        state = stateDetectWall;
      }
      break;

    case stateMarkMine:
//      Serial.println("stateMarkMine");
      markMine();
      state = stateDetectWall;
      break;

    case stateDetectWall:
      if (detectWall()){
//        Serial.println("Turning");

        if (!turnDirection == neutral){
          stop();
        }
        
        initialHeading = heading();
        projectedHeading = wrap(initialHeading, 180);
        turnDirection = switchTurnDirection(turnDirection);
        state = stateTurnAround;
      }
      else{
        state = stateBreak;
        turnDirection = neutral;
      }
      break;

    case stateTurnAround:
//      Serial.println("stateTurnAround");    
      if (!((projectedHeading-10) <= heading() and heading() <= (projectedHeading-10))){
        ("Targeted heading reached");
        stop();
      }
      else{
        if (turnDirection == clockwise){
//          Serial.println("turning clockwise");
          steer(speed1, 0);
        }

        else if (turnDirection == counterclockwise){
//          Serial.println("turning counterclockwise");
          steer(0, speed1);
        }
      }

      
      state = stateBreak;
      break;

    case stateBreak:
//      Serial.println("stateBreak");    
      if (minesDetected >= 8){
        state = stateStop;
      }
      else{
        if (turnDirection == neutral){
          steer(40, 40);
        }
        
        state = stateDetectMine;
      }
      break;
    
    case stateStop:
//      Serial.println("stateStop");
      break;
  }
  mpu.resetFIFO();
}
