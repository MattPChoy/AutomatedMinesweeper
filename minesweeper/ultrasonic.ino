int ultrasonic_distance(){
  int US_distance = sonar.convert_cm(sonar.ping_median(5));
  
  if (US_distance > 0){ // is valid
//    Serial.println(US_distance);
    return US_distance;
  }

//  while (US_distance == 0){
//    US_distance = sonar.convert_cm(sonar.ping_median(5));
//  }

//  return US_distance;
  else{
    // invalid distance
    return 999;
  }
}

bool detectWall(){
  if (ultrasonic_distance() <= minWallDist){
    Serial.print("Wall detected at");
    Serial.print(minWallDist);
    Serial.print("cm");
    return true;
  }
  else{
    return false;
  }
}
