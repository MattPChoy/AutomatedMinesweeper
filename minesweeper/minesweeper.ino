
/* Import External Packages */
#include <NewPing.h>
#include <Wire.h>
#include <Servo.h>
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

    #define forward HIGH
    #define reverse LOW
    #define left 0
    #define right 1

//    #define speed1 150
//    #define turnSpeed 80

    #define speed1 100
    #define turnSpeed 70

    #define triggerPin 7
    #define echoPin 6
    #define minWallDist 25
    #define range 3000 // the maximum distance allowed for the ultrasonic sensor

    #define interruptPin 2

    #define servo_init_angle  0
    #define servo_fire_angle  130
    #define button 6
    #define potpin  0 //analog in 0
    #define val 4

    int mineSensitivity = 100;

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
    bool runyet = false; // status of runonce
    bool minemarked = false;

    // states for mark mine state machine
    #define stateDetectMine 1000
    #define stateMinePause 1001
    #define stateMarkMine 1100
    #define stateMoveAroundMine 1200
    
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

    int currentCase, currentHeading;

    #define clockwise 1
    #define counterclockwise 2
    #define startbtn A3
    #define altbtn A2

/* Creation of Objects */
  // New Ping Library
    NewPing sonar(triggerPin, echoPin, range);
  // Adafruit HMC8553 Library
    Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
  // Servo motor library
    Servo myservo;


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
  stateMine = stateMinePause; // start the mark mines state machine with detection
}

void loop(){

  altbutton();
  startbutton();

  printState(state);
  printState(stateMine);
  switch(stateMine){
    case stateMinePause:
      break;
    
    case stateDetectMine:
      bool mine = detectMine();
      if (mine){
        Serial.println("mine detected");
        markMine();
        Serial.println("marked");
        stateMine = stateMarkMine;
        state = statePause;
        break;
      }
      else{
        stateMine = stateDetectMine;
        if (state == statePause){
          state = stateDetectWall;
        }
      }
      break;

    case stateMarkMine:
      markMine();
      stateMine = stateMoveAroundMine;
      break;


    case stateMoveAroundMine:
      if (move_around_mine()){
        // Returns true, moving aroudn mine is complete;
        stateMine = stateDetectMine;
      }
      break;
      
  }

  switch(state){
    case statePause:
      stop();
      break;

    case stateDetectWall:
      if (detectWall()){
        initialHeading = heading();
        projectedHeading = wrap(initialHeading, 180.0);
//        Serial.print("initialHeading:");
//        Serial.print(initialHeading);
//
//        Serial.print("projectedHeading:");
//        Serial.print(projectedHeading);

        turnDirection = switchTurnDirection(turnDirection);
        state = stateStartTurn;
//        Serial.println("stateStartTurn");
      }
      
      else{
        steer(speed1, speed1);
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
//      Serial.println("stateEvaluateHeading");
      break;

    case stateEvaluateHeading:
      // formerly stateTurnAround
      int currentHeading = heading();
//      Serial.println(currentHeading);
      if ((((projectedHeading-10) <= currentHeading) && (currentHeading <= (projectedHeading+10)))){
        state = stateDetectWall;
//        Serial.println("stateBreak");
//        stop();
        steer(speed1, speed1);
      }
      else{
        // do nothing, state remains the same until it reaches correct rotation
      }
      break;

    case stateBreak:
      if (minesDetected >= 8){
        state = stateStop;
//        Serial.println("stateStop");
      }
      else{
        state = stateDetectWall;
//        Serial.println("stateDetectWall");
      }
      break;

    case stateStop:
      // do nothing. Idle until reset.
      break;
  }


  mpu.resetFIFO();
}
