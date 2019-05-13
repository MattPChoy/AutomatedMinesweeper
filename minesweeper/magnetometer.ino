void displaySensorDetails(void){
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void getMagneticField(){
  sensors_event_t event; 
  mag.getEvent(&event);

  magx = event.magnetic.x;
  magy = event.magnetic.y;
  magz = event.magnetic.z;
}

bool detectMine(){
  if (magx > mineSensitivity or magy > mineSensitivity or magz > mineSensitivity){
    minesDetected += 1;
    return true;
  }

  else{
    return false;
  }
}
