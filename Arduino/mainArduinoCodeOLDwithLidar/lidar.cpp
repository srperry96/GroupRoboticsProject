/* Functions to deal with the LIDAR. LIDAR was scrapped late on in the project,
this is included for completeness.
Written by Robert Woolley (modifications by Samuel Perry to make it useable as
a library rather than a standalone piece of arduino code).  */

#include "lidar.h"

Lidar::Lidar(){
      // turns on servo which will make it under tension and sets the angle to 90
      servo.attach(servoPin);
      Wire.begin();
      servo.write(90);

      //changes the address of sensor2 while sensor1 is off
      pinMode(XSHUT_pin1, OUTPUT);
      digitalWrite(XSHUT_pin1, LOW);
      sensor2.setAddress(Sensor2_newAddress);
      pinMode(XSHUT_pin1, INPUT);
      delay(10);

      // initializes both lasers
      sensor1.init();
      sensor1.setTimeout(500);
      sensor2.init();
      sensor2.setTimeout(500);
}

void Lidar::setLongRange (bool long_range){
  // choice between long and short range
  if (long_range) {
    // lower the return signal rate limit (default is 0.25 MCPS)
    sensor1.setSignalRateLimit(0.1);
    sensor2.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  } else {
    //(default is 0.25 MCPS)
    sensor1.setSignalRateLimit(0.25);
    sensor2.setSignalRateLimit(0.25);
    //(defaults are 14 and 10 PCLKs)
    sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
    sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
    sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
    sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
  }
}

void Lidar::setTimingBudget (long budget){
  // set time allowance for each measurement
  // Default = 33ms = 33000, High speed = 20ms = 20000, High accuracy = 200 ms = 200000
  sensor1.setMeasurementTimingBudget(budget);
  sensor2.setMeasurementTimingBudget(budget);
}

void Lidar::fullScan(uint8_t increment){
  // completes a 360 degree scan at a given increment

  // if angle is small starts scan from 0 to 180
  sensor1.startContinuous();
  sensor2.startContinuous();
  if (current_angle <= 90){

    servo.write(0);     // Turn SG90 servo Right to 135 degrees
    delay(10*current_angle);


    for(int servoAngle = 0; servoAngle < 180; servoAngle = servoAngle + increment) { //move the micro servo from 0 degrees to 180 degrees
      servo.write(servoAngle);

      int reading1 = (sensor1.readRangeContinuousMillimeters()+5)/10;
      int reading2 = (sensor2.readRangeContinuousMillimeters()+5)/10;
      if (reading1 > 900) reading1 = 251; //time out
      else if (reading1 > 200) reading1 = 250; // out of range
      if (reading2 > 900) reading2 = 251; //time out
      else if (reading2 > 200) reading2 = 250;// out of range
      newScan[servoAngle] = reading1;
      newScan[servoAngle+180] = reading2;
    }
    servo.write(180);
    current_angle = 180;
  } else { // if the angle is big starts scan from 180 to
    servo.write(180);
    delay(10*(180-current_angle));

    for(int servoAngle = 179; servoAngle >= 0; servoAngle = servoAngle - increment) { //move the micro servo from 0 degrees to 180 degrees
      servo.write(servoAngle);

      int reading1 = (sensor1.readRangeContinuousMillimeters()+5)/10;
      int reading2 = (sensor2.readRangeContinuousMillimeters()+5)/10;
      if (reading1 > 900) reading1 = 251; //time out
      else if (reading1 > 200) reading1 = 250; // out of range
      if (reading2 > 900) reading2 = 251; //time out
      else if (reading2 > 200) reading2 = 250; // out of range
      newScan[servoAngle] = reading1;
      newScan[servoAngle+180] = reading2;
    }
    servo.write(0);
    current_angle = 0;
  }
  sensor1.stopContinuous();
  sensor2.stopContinuous();

  //load new scan into the scan array now that it has finished
  for(int i = 0; i < 360; i++){
      scan[i] = newScan[i];
  }
}

void Lidar::turnToAngle(uint8_t angle){
  if(!moving){
    if(angle < 180){
      servo.write(angle);
      current_angle = angle;
    } else {
      servo.write(angle-180);
      current_angle = angle-180;
    }
    moving = true;
    startMillis = millis();
  } else {
    //1 second delay without using the delay function and throwing off other timed elements
    currentMillis = millis();
    if((currentMillis - startMillis) >= 1000){
      moving = false;
    }
  }
}

void Lidar::singleScan(uint8_t sensorNum){
  // does a single scan at a given angle
  if (sensorNum == 1){ // if the angle is small it uses the front laser
    singleScanReading = (sensor1.readRangeSingleMillimeters()+5)/10;
  } else { // if the angle is big it uses the back laser
    singleScanReading = (sensor2.readRangeSingleMillimeters()+5)/10;
  }
}
