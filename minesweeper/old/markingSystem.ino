void markMine(){
    myservo.write(servo_offset+servo_fire_angle);    
    delay(90); 
    set_servo_via_pot(servo_offset);
    //myservo.write(servo_init_angle); 
    // sets the servo position according to the scaled valu18
    Serial.println("fire");

    delay(1000);

    Serial.println("marking finished");
}
