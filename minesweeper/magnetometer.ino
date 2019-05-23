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
  
  magx = event.magnetic.x - magxoffset;
  magy = event.magnetic.y - magyoffset;
  magz = event.magnetic.z - magzoffset;

  if (!magrunyet){
    magrunyet = true;
    magxoffset = magx;
    magyoffset = magy;
    magzoffset = magz;
  }

//  Serial.print(magx);
//  Serial.print(" | ");
//  Serial.print(magy);
//  Serial.print(" | ");
//  Serial.print(magz);

//  Serial.print(" ----- ");
//  Serial.print(magx+magxoffset);
//  Serial.print(" | ");
//  Serial.print(magy+magyoffset);
//  Serial.print(" | ");
//  Serial.println(magz+magzoffset);
}

bool detectMine(){
  getMagneticField();
  if (magrunyet){
    if (int(abs(magx)) > mineSensitivity){
      Serial.println("Mine detected");
//      return true;
    }
  
    else if (int(abs(magy)) > mineSensitivity){
      Serial.println("Mine detected");
//      return true;
    }
  
    else if (int(abs(magz)) > mineSensitivity){
      Serial.println("Mine detected");
//      return true;
    }
    else{
  //    Serial.println("Mine not detected!");
      return false;
    }
  }
}
