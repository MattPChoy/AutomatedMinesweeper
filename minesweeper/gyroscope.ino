float heading(){
  if (!dmpReady) return;
      while (!mpuInterrupt && fifoCount < packetSize) {
          if (mpuInterrupt && fifoCount < packetSize) {
            // try to get out of the infinite loop
            fifoCount = mpu.getFIFOCount();
          }
      }

      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();
      fifoCount = mpu.getFIFOCount();

      if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
          mpu.resetFIFO();
          fifoCount = mpu.getFIFOCount();
          Serial.println(F("FIFO overflow!"));
      }

      else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

          mpu.getFIFOBytes(fifoBuffer, packetSize);
          fifoCount -= packetSize;
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          mpu.resetFIFO();
          return int(((ypr[0] * 180/M_PI)+180+initial_heading)); // [0, 360]
    }
}

void dmpDataReady() {
    mpuInterrupt = true;
}

void calibrateGyro(){
  heading();  
}


bool gyro_is_ready(){
  
  gyro_is_ready_previous = gyro_is_ready_current;
  gyro_is_ready_current = heading();

  Serial.print(gyro_is_ready_current);

  int delta = abs(gyro_is_ready_previous - gyro_is_ready_current);

  if (delta > gyro_calibration_value){
    Serial.println("gyroscope is ready");
//    led_on();
    return true;
  }

  else{
    Serial.println("gyroscope is not ready");
    led_off();
    return false;
  }
}
