#include "gripper.h"

//Setup the gripper
void setup_gripper() {
  Dynamixel.begin(1000000, 4);
  delay(1000);
  //When the gripper is on power, reset the gripper
  reset_gripper();
}

//Reset the gripper
void reset_gripper() {
  //Disable continuous mode servomotor rotation
  Dynamixel.setEndless(1,OFF);
  Dynamixel.setEndless(3,OFF);
  //Start position
  move_overall(START_ID1, START_ID2, START_ID3, START_ID4);
  delay(1000);
}

//When the bear is on the floor, get the claws (pi camera) down.
//The angle between the camera and the horizontal line is 25 degrees.(-15 - 10)
void camera_down() {
  single_servo(4, START_ID4 - 10);
}

//When the bear is on the red car, get the claws (pi camera) back to center (the horizontal line)
void camera_center() {
  single_servo(4, START_ID4 + 15);
}

//Run only a single servo
void single_servo(int id, float degree_servo) {
  int position_servo;
  position_servo = (int)((degree_servo+150) * 10.24 / 3) ;
  Dynamixel.moveSpeed(id, position_servo, MOVE_SPEED);
  delay(2000);
}

//Four sevor motos work together
//ID=2 is Base; ID=4 is Shoulder; ID=1,3 are Wrists.
void move_overall(float degree_id1, float degree_id2, float degree_id3, float degree_id4) {
  int position_id[4];
  //Change the degree to the position because in DynamixelSerial1 library there is no function for setting the angle of rotation of the motor.
  position_id[0] = (int)((degree_id1+150) * 10.24 / 3) ;
  position_id[1] = (int)((degree_id2+150) * 10.24 / 3) ;
  position_id[2] = (int)((degree_id3+150) * 10.24 / 3) ;
  position_id[3] = (int)((degree_id4+150) * 10.24 / 3) ;
  //Move the actuator to the position at a fixed speed
  Dynamixel.moveSpeed(2, position_id[1], MOVE_SPEED);
  Dynamixel.moveSpeed(4, position_id[3], MOVE_SPEED);
  delay(2000);
  Dynamixel.moveSpeedRW(1, position_id[0], MOVE_SPEED);
  Dynamixel.moveSpeedRW(3, position_id[2], MOVE_SPEED);
  Dynamixel.action();
  delay(2000);
}

//Call this function to grab the bear if it is on the floor
void floorTeddy() {
  //Set a maximum torque on servo ID1 and ID3
  Dynamixel.torqueStatus (1, ON) ;
  Dynamixel.setMaxTorque (1, MAX_TORQUE) ;
  Dynamixel.torqueStatus (3, ON) ;
  Dynamixel.setMaxTorque (3, MAX_TORQUE) ;

  //Move to a fixed position
  //move_overall(-20,0,20,0);
  //Inclination position to prepar to grab the bear
  move_overall(-20,70,20,50);

  //Enable continuous mode servomotor rotation to grab the bear
  Dynamixel.setEndless(1,ON);
  Dynamixel.setEndless(3,ON);
  Dynamixel.turn(1,RIGTH,TURN_SPEED);
  Dynamixel.turn(3,LEFT,TURN_SPEED);
  delay(2000);
}

//Call this function to grab the bear if it is on the red car
void carTeddy() {
  //Set a maximum torque on servo ID1 and ID3
  Dynamixel.torqueStatus (1, ON) ;
  Dynamixel.setMaxTorque (1, MAX_TORQUE) ;
  Dynamixel.torqueStatus (3, ON) ;
  Dynamixel.setMaxTorque (3, MAX_TORQUE) ;

  //Move to a fixed position
  move_overall(-20,0,20,0);
  //Inclination position to prepar to grab the bear
  move_overall(-20,25,20,30);

  //Enable continuous mode servomotor rotation to grab the bear
  Dynamixel.setEndless(1,ON);
  Dynamixel.setEndless(3,ON);
  Dynamixel.turn(1,RIGTH,TURN_SPEED);
  Dynamixel.turn(3,LEFT,TURN_SPEED);
  delay(2000);
  //Disable the torque of servo ID2 and ID4 to make the claws fall down on the floor
  Dynamixel.torqueStatus (2, OFF) ;
  Dynamixel.torqueStatus (4, OFF) ;
}
