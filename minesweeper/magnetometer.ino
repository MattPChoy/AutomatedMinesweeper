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

  
  Serial.print(abs(magx));
  Serial.print(" | ");
  
  Serial.print(abs(magy));
  Serial.print(" | ");
  
  Serial.println(abs(magz));
  
}

bool detectMine(){
  getMagneticField();

  if (abs(magx) > mineSensitivity){
    minesDetected += 1;
    Serial.print("Mine detected! X:");
    Serial.println(abs(magx));
    return true;
  }

  else if (abs(magy) > mineSensitivity){
    minesDetected += 1;
    Serial.println("Mine detected! Y:");
    Serial.println(abs(magy));
    return true;
  }

  else if (abs(magz) > mineSensitivity){
    minesDetected += 1;
    Serial.println("Mine detected! Z:");
    Serial.println(abs(magz));
    return true;
  }

  else{
    return false;
  }
}
