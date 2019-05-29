bool markMine(){
//  Serial.println("marking");
//  myservo.write(val+50);
//  delay(200);
//  myservo.write(val+servo_fire_angle);
//  delay(200);

  switch (markMineState){
    case 0:
        led_on();
        Serial.println("marking the mine");
        pwm.setPWM(0, 0, SERVOMIN);
        markMineState = 1;
        markMineMillis = millis();
        break;
    
    case 1:
      if (millis() > (markMineMillis+500)){
        pwm.setPWM(0, 0, SERVOMAX);
        markMineState = 2;
        markMineMillis = millis();
      }
      break;

    case 2:
      if (millis() > (markMineMillis+500)){
        markMineState = 3;
        led_off();
      }
      break;
    case 3:
      break;
  }

  if (markMineState == 3){
    return true;
  }

  else{
    return false;
  }

//  led_on();
//  pwm.setPWM(0, 0, SERVOMIN);
//  delay(500);  
//  pwm.setPWM(0, 0, SERVOMAX);
//  delay(500);
//  led_off();
//  mpu.resetFIFO();
}

void resetServo(){
  if (millis() > (initReset+500)){
    Serial.print("rst done");
    pwm.setPWM(0, 0, SERVOMIN);
    markMineState = 0;
  }
}
