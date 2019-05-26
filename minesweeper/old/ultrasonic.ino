int ultrasonic_distance(){
  int US_distance = sonar.convert_cm(sonar.ping_median(5));
  
  if (US_distance > 0){ // is valid
//    Serial.println(US_distance);
    return US_distance;
  }
  else{
    // invalid distance
    return 999;
  }
}

bool detectWall(){
  if (ultrasonic_distance() <= minWallDist){
    Serial.println("Wall detected <=10cm");
    return true;
  }
  else{
    return false;
  }
}
