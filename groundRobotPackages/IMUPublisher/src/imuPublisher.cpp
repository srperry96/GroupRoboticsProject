/* ROS node for publishing the yaw value determined using the IMU onboard the
ground robo.
Written by Samuel Perry
NOTE: Several elements of the code for using the IMU were taken from one of the demo
programs in the RTIMULib library. The code was adapted and added to in order to make
the yaw values publish to ROS. */

#include <stdio.h>
#include <stdint.h>

#include "ros/ros.h"

#include <RTIMULib.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"

int main(int argc, char ** argv){
	//first, set up the ros stuff for publishing
	ros::init(argc, argv, "groundRobotYawPublisher");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::Int16>("groundRobot/Yaw", 10);

	//now setup all the IMU stuff
	int sampleCount = 0;
  int sampleRate = 0;
  uint64_t rateTimer;
  uint64_t displayTimer;
  uint64_t now;

  RTIMUSettings *settings = new RTIMUSettings("RTIMULib");

  RTIMU *imu = RTIMU::createIMU(settings);

  if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
      printf("No IMU found\n");
      exit(1);
  }

  //initialise IMU
  imu->IMUInit();

  // set imu parameters
  imu->setSlerpPower(0.02);
  imu->setGyroEnable(true);
  imu->setAccelEnable(true);
  imu->setCompassEnable(true);

  //  set up for rate timer
  rateTimer = displayTimer = RTMath::currentUSecsSinceEpoch();
	//std_msgs::Float32 yawVal;
	std_msgs::Int16 yawVal;
	//loop forever, publishing yaw from imu
	while (ros::ok()) {
  	//  poll at the rate recommended by the IMU
    usleep(imu->IMUGetPollInterval() * 1000);

    while (imu->IMURead()) {
    	RTIMU_DATA imuData = imu->getIMUData();
      sampleCount++;

      now = RTMath::currentUSecsSinceEpoch();
      //publish 10 times a second
      if((now - displayTimer) > 100000){
      	yawVal.data = (int)(imuData.fusionPose.z() * RTMATH_RAD_TO_DEGREE);
        pub.publish(yawVal);

        displayTimer = now;
      }

      //  update rate every second
      if ((now - rateTimer) > 1000000) {
      	sampleRate = sampleCount;
        sampleCount = 0;
        rateTimer = now;
      }
    }
  }
	return 0;
}
