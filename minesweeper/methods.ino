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
