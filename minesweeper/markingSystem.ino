void markMine(){
//  Serial.println("marking");
//  myservo.write(val+50);
//  delay(200);
//  myservo.write(val+servo_fire_angle);
//  delay(200);
  Serial.println("marking the mine");
  pwm.setPWM(0, 0, SERVOMIN);
  delay(500);
  pwm.setPWM(0, 0, SERVOMAX);
  delay(500);
}

void resetServo(){
  pwm.setPWM(0, 0, SERVOMIN);
  delay(500);
}
