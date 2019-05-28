void set_motors(int side, int input_speed){
    // Step 1: Initialise Variables
    int PWM;
    int DIR;
    int motor_speed;
    boolean motor_direction;

    // Step 1: Assign pins to be written to 
    if (side == left){
        PWM = E2Pin;
        DIR = M2Pin;
    }

    else if (side == right){
        PWM = E1Pin;
        DIR = M1Pin;
    }

    // Step 2: Handle cases for +ve and -ve speeds
    if (input_speed == 0){
        motor_speed = 0;
        motor_direction = forward;
    }
    
    if (input_speed < 0){
        // negative or backwards
        motor_speed = input_speed * (-1);
        motor_direction = reverse;
    }

    else if (input_speed > 0){
        // forwards direction
        motor_direction = forward;
        motor_speed = input_speed;
    }


    // Step 3: Handle edge case of speed being greater than maximum value
    if (motor_speed > 255){
        motor_speed = 255;
    }

    // Step 4: Write speed and direction to motor via previously assigned pins.
    digitalWrite(DIR, motor_direction);
    analogWrite(PWM, motor_speed);
}

void steer(int left_speed, int right_speed){
    set_motors(left, left_speed);
    set_motors(right, right_speed);
}

void stop(){
    steer(0,0);
}

void pause(int some_delay){
    stop();
    delay(some_delay);
}

/*
void rotate(int angle){
  // rotate on the inner track
  // right: true
  // left: false

  mpu.resetFIFO();
  
  int initial_heading = heading();
  int target_heading = heading() + angle;
  
  if (turnDirection == false){
    while(!(((target_heading-10) <= heading()) and (heading <= (target_heading-10)))){
      steer(speed1, 0);
    }
    turnDirection = false;
    }
  else if (turnDirection == true){
    while(!(((target_heading-10) <= heading()) and (heading <= (target_heading-10)))){
      steer(0, speed1);
    }
    turnDirection = true;
  }
}
*/

bool move_around_mine(){
  int currentHeading = heading();
  switch(currentCase){
      case 1:
        Serial.println(" case1");
        // initiate left turn
        projectedHeading = currentHeading + 90;
        steer(turnSpeed, 0);
        currentCase = 2;
        Serial.print("current heading: ");
        Serial.print(currentHeading);

        Serial.print(" --- projected heading: ");
        Serial.print(projectedHeading);
        return false;
        break;
        

      case 2:
        Serial.println(" case2");
        if ((((projectedHeading-10) <= currentHeading) && (currentHeading <= (projectedHeading+10)))){
          currentCase = 3;
          stop();
        }
        return false;
        break;

      case 3:
        Serial.println(" case3");
        stop();
        currentCase =  4;
        return false;
        break;

      case 4:
        Serial.println(" case4");
        currentHeading = heading();
        projectedHeading = wrap(currentHeading, 180);
        steer(turnSpeed/(2.2), turnSpeed);
        currentCase = 5;
        return false;
        break;

      case 5:
        Serial.print(" case5");

        Serial.print("Current Heading: ");
        Serial.print(currentHeading);

        Serial.print("Projected Heading");
        Serial.println(projectedHeading);
        if ((((projectedHeading-10) <= currentHeading) && (currentHeading <= (projectedHeading+10)))){
          currentCase = 6;
          stop();
        }
        return false;
        break;

      case 6:
        Serial.println(" case6");
        // initiate left turn
        currentHeading = heading();
        projectedHeading = wrap(currentHeading, 90);

        Serial.print("Current Heading: ");
        Serial.print(currentHeading);

        Serial.print("Projected Heading");
        Serial.print(projectedHeading);
        
        steer(turnSpeed, 0);
        currentCase = 7;
        return false;
        break;
        

      case 7:
        Serial.println(" case7");
        if ((((projectedHeading-10) <= currentHeading) && (currentHeading <= (projectedHeading+10)))){
          currentCase = 8;
          stop();  
        }
        return false;
        break;

      case 8:
        Serial.println(" case8");
        stop();
        return true;
        break;
    }
}
