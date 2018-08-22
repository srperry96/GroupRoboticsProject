#include "gripper.h"

char inputByte = 0;

void setup_gripper() {
  Dynamixel.begin(1000000, 4);
  delay(2000);
  reset_gripper();
  Serial.println("Setup gripper");
}

//Rest the gripper
void reset_gripper() {
  Dynamixel.setEndless(1,OFF);
  Dynamixel.setEndless(3,OFF);
  move_overall(80,-45,-80,-45);
  delay(1000);

}

void camera_down() {
  single_servo(4,-55);
}

void single_servo(int id, float degree_servo) {
  int position_servo;
  position_servo = (int)((degree_servo+150) * 10.24 / 3) ;
  Dynamixel.moveSpeed(id, position_servo, MOVE_SPEED);
  delay(2000);
}


float pos_to_deg(int pos) {
  float deg;
  deg = pos * 3 / 10.24 - 150;
  Serial.println( deg);
  return deg;

}

void move_overall(float degree_id1, float degree_id2, float degree_id3, float degree_id4) {
  int position_id[4];
  position_id[0] = (int)((degree_id1+150) * 10.24 / 3) ;
  position_id[1] = (int)((degree_id2+150) * 10.24 / 3) ;
  position_id[2] = (int)((degree_id3+150) * 10.24 / 3) ;
  position_id[3] = (int)((degree_id4+150) * 10.24 / 3) ;
  Dynamixel.moveSpeed(2, position_id[1], MOVE_SPEED);
  Dynamixel.moveSpeed(4, position_id[3], MOVE_SPEED);
  delay(2000);
  Dynamixel.moveSpeedRW(1, position_id[0], MOVE_SPEED);
  Dynamixel.moveSpeedRW(3, position_id[2], MOVE_SPEED);
  Dynamixel.action();
  delay(2000);

}

//If the bear is on the floor
void floorTeddy() {

  Dynamixel.torqueStatus ( 1, ON) ;
  Dynamixel.setMaxTorque (1, MAX_TORQUE) ;
  Dynamixel.torqueStatus ( 3, ON) ;
  Dynamixel.setMaxTorque (3, MAX_TORQUE) ;

  //move to fixed position 1
  move_overall(-20,0,20,0);
  //inclination position
  move_overall(-20,70,20,50);

  Dynamixel.setEndless(1,ON); //In continuos mode
  Dynamixel.setEndless(3,ON);

  Dynamixel.turn(1,RIGTH,TURN_SPEED);
  Dynamixel.turn(3,LEFT,TURN_SPEED);

  // read positions of different servos
  // id=1,3 are Wrist; id=2 is Base; id=4 is Shoulder
  Serial.println( Dynamixel.readPosition(1));
  Serial.println( Dynamixel.readPosition (2));
  Serial.println( Dynamixel.readPosition (3));
  Serial.println( Dynamixel.readPosition (4));
  delay(2000);

}

//If the bear is on the red car
void carTeddy() {

  Dynamixel.torqueStatus ( 1, ON) ;
  Dynamixel.setMaxTorque (1, MAX_TORQUE) ;
  Dynamixel.torqueStatus ( 3, ON) ;
  Dynamixel.setMaxTorque (3, MAX_TORQUE) ;

  //move to fixed position 1
  move_overall(-20,0,20,0);
  //inclination position
  move_overall(-20,25,20,30);

  Dynamixel.setEndless(1,ON); //In continuos mode
  Dynamixel.setEndless(3,ON);

  Dynamixel.turn(1,RIGTH,TURN_SPEED);
  Dynamixel.turn(3,LEFT,TURN_SPEED);
  //Turn off servo ID2 and ID4 to make the gripper fall down on the floor
  delay(2000);
  Dynamixel.torqueStatus ( 2, OFF) ;
  Dynamixel.torqueStatus ( 4, OFF) ;

}
