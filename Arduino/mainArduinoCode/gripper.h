#ifndef __GRIPPER_H__
#define __GRIPPER_H__

#include <Arduino.h>
#include <DynamixelSerial1.h>

#define MOVE_SPEED 100
#define MAX_TORQUE 1023
#define TURN_SPEED 300

void setup_gripper();
void move_overall(float degree_id1, float degree_id2, float degree_id3, float degree_id4);
void reset_gripper();
void move_claws(float degree_id1,float degree_id3);
void floorTeddy();
void carTeddy();
void camera_down();
void single_servo(int id, float degree_servo);

#endif
