/*ROS node which handles all of the communication between the Raspberry Pi and the
Arduino in the ground robot.
Written by Samuel Perry.*/

/* Note: There is some casting from uint8_t to uint16_t in this file. This is because the
values are being published to a topic which is subscribed to by a node written in python. Python
doesn't handle unsigned integers, so we need the extra bits in order to store the numbers in such
a way that we can publish to ROS and have python understand them without losing data. */

#include <stdio.h>
#include <stdint.h>

#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Empty.h"

#include "spiComms.h"

//Publishers for IR and laser time of flight sensor readings
ros::Publisher irPub;
ros::Publisher laserPub;

/* Callback function for the IR reading request subscriber. Gets a set of IR readings from
the Arduino and publishes them */
void requestIRReadingsCallback(const std_msgs::Empty& msg){
	std_msgs::Int8MultiArray irReadings;

	//get IR values from the Arduino through SPI
	irGetValues();

	//clear irReadings array (just to be sure)
	irReadings.data.clear();

	//copy the ir data into irReadings to be published
	for(int i = 0; i < 8; i++){
		irReadings.data.push_back(irValues[i]);
	}

	//publish the data
	irPub.publish(irReadings);
}

/* Callback for any gripper instructions. Calls the corresponding gripper command */
void gripperInstructionsCallback(const std_msgs::Int8::ConstPtr& msg){
	switch(msg->data){
		//reset the gripper to its start position
		case 0:	armReset();
						break;

		//grab the teddy high (on top of the car)
		case 1:	armGripHigh();
						break;

		//grab the teddy low (on the ground)
		case 2:	armGripLow();
						break;

		//tilt the arm camera down
		case 3:	armTiltCameraLow();
						break;

		//tilt the arm camera to its centered position so we can clearly see the bear on top of the car
		case 4: armTiltCameraCenter();
						break;

		default: break;
	}
}

/* Callback function which gets the most recent laser sensor reading from the arduino, then publishes it */
void requestLaserReadingCallback(const std_msgs::Empty& msg){
	std_msgs::Int16 laserReading;
	//cast to 16 bit uint here as python doesnt do unsigned ints ie uses the MSB as a sign, so we lose info if we dont cast
	laserReading.data = (uint16_t)laserGetReading();

	laserPub.publish(laserReading);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "groundRobotSPIHandler");

	ros::NodeHandle nh;

	//Publisher for IR readings - 8 sensors = 8 element array
	irPub = nh.advertise<std_msgs::Int8MultiArray>("/groundRobot/IRReadings", 10);

	//Publisher for the laser sensor readings
	laserPub = nh.advertise<std_msgs::Int16>("/groundRobot/LaserReadings", 10);

	//Subscriber for topic which tells this node to get a set of IR readings from the Arduino
	ros::Subscriber requestIRReadingsSubscriber = nh.subscribe("/groundRobot/RequestIRReadings", 10, requestIRReadingsCallback);

	//Subscriber for any gripper instructions
	ros::Subscriber gripperInstructionsSubscriber = nh.subscribe("/groundRobot/GripperInstructions", 0, gripperInstructionsCallback);

	//Subscriber used to request a laser sensor reading
	ros::Subscriber laserReadingRequestSubscriber = nh.subscribe("/groundRobot/RequestLaserReading", 10, requestLaserReadingCallback);

	//setup SPI connection and test it with a basic handshake. If this fails, return 0
	if(!setupSPIComms()){
		return 0;
	};

	//start ROS functionality
	ros::spin();

	return 0;
}
