#include "ros/ros.h"

#include <sstream>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"

#define LEFT 3
#define RIGHT 2

std_msgs::Int16MultiArray controlInstruction;

void moveStraight(int speed){
  //limit speed
  if(speed > 10) speed = 10;
  else if(speed < -10) speed = -10;
  //set data to be published
  controlInstruction.data.clear();
  controlInstruction.data.push_back(1);
  controlInstruction.data.push_back(speed);
}

void stop(){
  controlInstruction.data.clear();
  controlInstruction.data.push_back(4);
  //second parameter doesnt matter, but needs to be filled, so just set to 0
  controlInstruction.data.push_back(0);
}

void turn(int direction, int angle){
  controlInstruction.data.clear();
  controlInstruction.data.push_back(direction);
  controlInstruction.data.push_back(angle);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "piMovementControlPublisher");

  ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<std_msgs::Int16MultiArray>("piMovementControl", 10);

  while(ros::ok()){
    turn(LEFT, 180);
    //publish to the topic
    pub.publish(controlInstruction);
    ros::spinOnce();
    getchar();

    moveStraight(10);
    //publish to the topic
    pub.publish(controlInstruction);
    ros::spinOnce();
    getchar();

    stop();
    //publish to the topic
    pub.publish(controlInstruction);
    ros::spinOnce();
    getchar();

    turn(RIGHT, 180);
    //publish to the topic
    pub.publish(controlInstruction);
    ros::spinOnce();
    getchar();

  }

  return 0;
}
