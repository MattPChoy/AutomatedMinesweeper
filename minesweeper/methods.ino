void wait_for_start(){
  while(digitalRead(startbtn) == HIGH){
    mpu.resetFIFO();
  }
}

void startbutton(){
  if (digitalRead(startbtn) == LOW){
    state = stateDetectWall;
    Serial.println("Start!");
  }
}

void altbutton(){
  if (digitalRead(altbtn) == LOW){
    state = statePause;
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

void set_servo_via_pot(int val){

  myservo.write(servo_offset);
  Serial.print("servo angle:");
  Serial.print(servo_offset);
}

void moveAroundMine(){
  steer(speed1, speed1);
  delay(500);
}

void tuneServo(){
//  init_marker();
//  pinMode(button,INPUT_PULLUP);
//    int val= analogRead(potpin);
//  val=map(val,0,1023,0,180);
//  Serial.begin(115200);
//  Serial.println("begin");
//  set_servo_via_pot(servo_offset);
}
