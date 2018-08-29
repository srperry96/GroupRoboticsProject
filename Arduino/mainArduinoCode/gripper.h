/* Functions relating to the arm / gripper of the ground robot
Written by Tingting Yan (Modifications by Samuel Perry to make the code useable as
a library rather than its own standalone arduino code)*/

#ifndef __GRIPPER_H__
#define __GRIPPER_H__

#include <Arduino.h>
#include <DynamixelSerial1.h>

//Set servo motor rotation speed to be 100. The range is 0 to 1023.
#define MOVE_SPEED 100
//Set a torque on servo ID1 and ID3 to be 1023. The range is 0 to 1023.
#define MAX_TORQUE 1023
//Set turn speed of servo ID1 and ID3 in continuous mode to be 300
#define TURN_SPEED 300
//Start degrees of servo ID1. The range is -150 to +150.
#define START_ID1 90
//Start degrees of servo ID2. The range is -150 to +150.
#define START_ID2 -45
//Start degrees of servo ID3. The range is -150 to +150.
#define START_ID3 -90
//Start degrees of servo ID4. The range is -150 to +150. '-45' of servo 4 means the pi camera looks forwared horizontally. '-60' means the pi camera looks down 15 degrees.
//That's because at 15 degrees, the ground robot can see the bear both on the floor and on the car in the distance of 40 centimeters.
#define START_ID4 -60


//Setup the gripper
void setup_gripper();
//Rest the gripper
void reset_gripper();
//When the bear is on the floor, get the claws (pi camera) down.
void camera_down();
//When the bear is on the red car, get the claws (pi camera) back to center (the horizontal line)
void camera_center();
//Run only a single servo
void single_servo(int id, float degree_servo);
//Four sevor motos work together
void move_overall(float degree_id1, float degree_id2, float degree_id3, float degree_id4);
//Call this function to grab the bear if it is on the floor
void floorTeddy();
//Call this function to grab the bear if it is on the red car
void carTeddy();


#endif
