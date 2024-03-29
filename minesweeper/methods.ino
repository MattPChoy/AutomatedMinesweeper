void printState(int state){
  /*
   *Current State Definitions
    #define stateDetectWall 10
    #define stateStartTurn 20
    #define stateEvaluateHeading 30
    #define stateBreak 40
    #define stateStop 50
    #define statePause 60
  */ 

  /*
   *Current State Mine Definitions
    #define stateDetectMine 1000
    #define stateMinePause 1001
    #define stateMarkMine 1100
    #
  */
  if (true){
    if (state == stateDetectWall){
      Serial.println("stateDetectWall");
    }
    else if (state == stateStartTurn){
      Serial.println("stateStartTurn");
    }
    else if (state == stateEvaluateHeading){
      Serial.println("stateBreak");
    }
    else if (state == stateStop){
      Serial.println("stateStop");
    }
    else if (state == statePause){
      Serial.println("statePause");
    }
  
    if (state == stateDetectMine){
      Serial.println("stateDetectMine");
    }
    else if (state == stateMinePause){
      Serial.println("stateMinePause");
    }
    else if (state == stateMarkMine){
      Serial.println("stateMarkMine");
    }
    else if (state == 60){
      Serial.println("det");
    }
    else{
      Serial.print("unassigned ");
      Serial.println(state);
    }
  }
}

void wait_for_start(){
  while(digitalRead(startbtn) == HIGH){
    mpu.resetFIFO();
    Serial.println(heading());
    calibrateGyro();
  }
}

void startbutton(){
  if (digitalRead(startbtn) == LOW){
    Serial.print("start");
    state = stateDetectWall;
    stateMine = stateDetectMine;
  }
}

void altbutton(){
  if (digitalRead(altbtn) == LOW){
    pretest();
  }
}

void pretest(){
  // move forward
  steer(speed1, speed1);
  delay(1000);

  // move backward
  steer(-speed1, -speed1);
  delay(1000);

  
  // turn left
  steer(turnSpeed, -turnSpeed);

  currentHeading = heading();
  projectedHeading = wrap(currentHeading, 90);

  delay(2000);

  steer(-turnSpeed, turnSpeed);

  while (!((projectedHeading-10) <= currentHeading) && (currentHeading <= (projectedHeading+10))){
    currentHeading = heading();
    projectedHeading = wrap(currentHeading, 180);
  }
  
  stop();

  // turn right
  
  // detect mine
  while(1){
    if (detectMine()){
      Serial.println("mine detected");
      markMine();
    }
  }

  while(1){
    resetServo();
  }
}

int wrap(int position1, int position2){
  int sum = position1 + position2;

  while (sum > 360){
    sum -= 360;
  }

  while (sum < 0){
    sum += 360;
  }

  return sum;
}

int switchTurnDirection(int funcTurnDirection){
  if (funcTurnDirection == clockwise){
    return counterclockwise;
  }

  else if (funcTurnDirection == counterclockwise){
    return clockwise;
  }
}


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
//  Serial.println("HMC5883 Magnetometer Initialised");
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

//  pinMode(LED1, OUTPUT);
}

void led_on(){
  digitalWrite(LED1, HIGH);
}

void led_off(){
  digitalWrite(LED1, LOW);
}

void init_marking(){
//  myservo.attach(5);
//  myservo.write(servo_init_angle);
  pwm.begin();
  pwm.setPWMFreq(60);
  delay(10);
}
