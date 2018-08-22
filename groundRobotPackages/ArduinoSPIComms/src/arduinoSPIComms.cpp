/* Note: There is a lot of casting from uint8_t to uint16_t in this file. This is because the
values are being published to a topic which is subscribed to by a node written in python. Python
doesn't handle unsigned integers, so we need the extra bits in order to store the numbers in such
a way that we can publish to ROS and have python understand them. */

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

//publishers - declared globally as they are initialised in main, then used in callback functions
ros::Publisher irPub;
ros::Publisher lidarFullScanPub;
ros::Publisher lidarSingleScanPub;

/* Publish a full set of lidar readings (360 values), along with the increment size in array element 360 */
void publishLidarFullScan(){
	std_msgs::Int16MultiArray temp;

	//load lidar scan data into temp so it can be published
	for(int i = 0; i < 360; i += lastScanIncrement){
		temp.data.push_back((uint16_t)fullScanData[i]);
	}

	//add 252 to mark the end of the data in the array
	temp.data.push_back(252);

	//add increment to the final element of the array
	temp.data.push_back((uint16_t)lastScanIncrement);

	//publish the array
	lidarFullScanPub.publish(temp);
}


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

/* Callback function handling all lidar SPI commands. Calls the lidar function corresponding
to the value in element 1 of the message */
void lidarSPIInstructionsCallback(const std_msgs::Int16MultiArray::ConstPtr& msg){
	std_msgs::Int16 temp;

	//first element of array determines what instruction to run, second element is the parameter (if required)
	switch(msg->data[0]){
		//trigger a full lidar scan
		case 1:	lidarStartFullScan(msg->data[1]);
						break;

		//retrieve the most recent set of lidar data from the arduino, then publish it
		case 2: lidarGetFullScanData();
						publishLidarFullScan();
						break;

		//move to a specific angle
		case 3:	lidarGoToAngle(msg->data[1]);
						break;

		//get a single lidar scan result and publish it
		case 4:	temp.data = (uint16_t)lidarSingleScan((uint8_t)msg->data[1]);
						lidarSingleScanPub.publish(temp);
						break;

		//set the range (long or short) for the lidar
		case 5:	lidarSetRange(msg->data[1]);
						break;

		//set the time used per scan for the sensors in the lidar
		case 6:	lidarSetTimingBudget(msg->data[1]);
						break;

		default:break;
	}
}

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

		//tilt the arm camera down (there is no tilt up as we can just use the reset function instead)
		case 3:	armTiltCameraLow();
						break;

		default: break;
	}
}


int main(int argc, char **argv){
	ros::init(argc, argv, "groundRobotSPIHandler");

	ros::NodeHandle nh;

	//Publisher for IR readings - 8 sensors = 8 element array
	irPub = nh.advertise<std_msgs::Int8MultiArray>("/groundRobot/IRReadings", 10);

	//Publisher for full lidar scan data set
	lidarFullScanPub = nh.advertise<std_msgs::Int16MultiArray>("/groundRobot/LidarFullScanReadings", 10);

	//Publisher for single lidar scan reading
	lidarSingleScanPub = nh.advertise<std_msgs::Int16>("/groundRobot/LidarSingleScanReading", 10);


	//Subscriber for topic which tells this node to get a set of IR readings from the Arduino
	ros::Subscriber requestIRReadingsSubscriber = nh.subscribe("/groundRobot/RequestIRReadings", 10, requestIRReadingsCallback);

	//Subscriber for any LIDAR instructions
	ros::Subscriber lidarSPIInstructionsSubscriber = nh.subscribe("/groundRobot/LidarSPIInstructions", 10, lidarSPIInstructionsCallback);

	//Subscriber for any gripper instructions
	ros::Subscriber gripperInstructionsSubscriber = nh.subscribe("/groundRobot/GripperInstructions", 10, gripperInstructionsCallback);

	//setup SPI connection and test it. If this fails, return 0
	if(!setupSPIComms()){
		return 0;
	};


	//start ROS functionality
	ros::spin();


	return 0;
}
