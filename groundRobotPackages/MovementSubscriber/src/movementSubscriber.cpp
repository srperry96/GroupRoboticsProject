//COMMENTED INCLUDES ARE IN groundRobotMotorControl.h
//#include <stdio.h>
//#include <stdint.h>
//#include <pigpiod_if2.h> //gpio pin library for hardware pwm etc

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "groundRobotMotorControl.h"

int piID, nothingReceivedCounter;

/* Movement subscriber callback. Translates twist messages into corresponding left and right
wheel speeds and sets the accordingly */
void groundMovementControlCallback(const geometry_msgs::Twist::ConstPtr& msg){

	float rightSpeed = 0, leftSpeed = 0;

	//if we dont want to move, brake
	if((msg->linear.x == 0) && (msg->angular.z == 0)){
		brake();
	//else we want to move
	}else{
		//if turning on the spot (linear is 0, but turn is not)
		if(msg->linear.x == 0){
			rightSpeed = msg->angular.z;
			leftSpeed = -msg->angular.z;
		//else we are moving
		}else{
			//set linear speeds
			rightSpeed = msg->linear.x;
			leftSpeed = msg->linear.x;

			//now calculate difference in speeds based on amount of turn (if necessary)
			//turning anticlockwise (left)
			if(msg->angular.z > 0){
				leftSpeed -= 2 * msg->angular.z * leftSpeed;
			//turning clockwise (right)
			}else if(msg->angular.z < 0){
				rightSpeed += 2 * msg->angular.z * rightSpeed;
			}
		}
		//set pwm values based on calculated speeds
		setPWM(LEFTWHEEL, leftSpeed);
		setPWM(RIGHTWHEEL, rightSpeed);
	}

	//reset counter to 0 as we have received an instruction
	nothingReceivedCounter = 0;
}

void nothingReceivedTimerCallback(const ros::TimerEvent&){
	nothingReceivedCounter++;
	//if no instructions have been received for over 1 second, stop the robot moving
	if(nothingReceivedCounter > 10){
		brake();
	}
}

int main(int argc, char **argv){

	//setup the base motors
	setupMotors();

	//initialise node called groundRobotMovementControlListener
	ros::init(argc, argv, "groundRobotMovementControlListener");
	ros::NodeHandle nh;

	//subscribe to the movement control topic
	ros::Subscriber movementControlSubscriber = nh.subscribe("/groundRobot/MovementControl", 10, groundMovementControlCallback);

	//timer which will stop the robot if no instruction is received for a set period of time (probably because the network is down)
	ros::Timer nothingReceivedTimer = nh.createTimer(ros::Duration(0.1), nothingReceivedTimerCallback);

	//start
	ros::spin();

	//make sure to stop moving when the node is stopped
	brake();

	return 0;
}
