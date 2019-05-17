/* Import External Packages */
#include <NewPing.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/* Variable Definitions */
  // #define
    #define baudRate 115200

    #define E1Pin 10 //PWM-1
    #define E2Pin 11 //PWM-2
    #define M1Pin 12 //DIR-1
    #define M2Pin 13 //DIR-2

    #define forward LOW
    #define reverse HIGH
    #define left 0
    #define right 1

    #define speed1 100
    #define turnSpeed 80

    #define triggerPin 7
    #define echoPin 6
    #define minWallDist 10
    #define range 3000 // the maximum distance allowed for the ultrasonic sensor

    #define interruptPin 2

    #define mineSensitivity 400

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
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    unsigned long initTime = 0;
    unsigned long debounceDelay = 1000;
    int l_speed, r_speed;
    int initial_heading;
    bool start = false; // status of run
    bool runyet = false; // status of runoncebool dmpReady = false;  // set true if DMP init was successful

    // states for mark mine state machine
    #define stateDetectMine 1000
    #define stateMarkMine 1100
    int stateMine;

    // states for main state machine
    #define stateDetectWall 10
    #define stateStartTurn 20
    #define stateEvaluateHeading 30
    #define stateBreak 40
    #define stateStop 50
    #define statePause 60
    int state;

    // variables for state machine layer 1
    int initialHeading;
    int projectedHeading;
    int turnDirection;

    #define clockwise 1
    #define counterclockwise 2
    #define startbtn A3
    #define altbtn A2

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

      if (devStatus == 1){
        Serial.print("Restart the Arduino for a quick fix");
      }
  }
}

void init_components(){
  pinMode(startbtn, INPUT);
  digitalWrite(startbtn, HIGH);

  pinMode(altbtn, INPUT);
  digitalWrite(altbtn, HIGH);
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
  // wait_for_start(); // clear fifo while in block operation

  Serial.println("Starting motors");

    turnDirection = counterclockwise;

  state = statePause; // start the state machine with the initial state
  stateMine = stateDetectMine; // start the mark mines state machine with detection
}

void loop(){
  /*
   * Write some code here that utilises a switch case to mark the mines and
   * upon finishing, returns to the main switch case. This can not be blocking
   * code as the fifo will overflow and cause errors with the gyroscope.
  */

  altbutton();
  startbutton();

  switch(state){
    case stateDetectWall:
      if (detectWall()){
        initialHeading = heading();
        projectedHeading = wrap(initialHeading, 180.0);
        turnDirection = switchTurnDirection(turnDirection);
        state = stateStartTurn;
      }
      break;

    case stateStartTurn:
      if (turnDirection == clockwise){
        steer(turnSpeed, 0);
      }

      else if(turnDirection == counterclockwise){
        steer(0, turnSpeed);
      }

      state = stateEvaluateHeading;
      break;

    case stateEvaluateHeading:
  f    
  }


  mpu.resetFIFO();
}
