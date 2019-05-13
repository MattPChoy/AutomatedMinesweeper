void testMotors(){
  steer(60,60);
}

void testUltrasonic(){
  int usd = ultrasonic_distance();
  Serial.println(usd);
}
