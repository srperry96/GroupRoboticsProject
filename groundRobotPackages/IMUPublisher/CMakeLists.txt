cmake_minimum_required(VERSION 2.8.3)
project(IMUPublisher)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS}
	/home/pi/Desktop/GRP/RTIMULib/RTIMULib
)

add_subdirectory(/home/pi/Desktop/GRP/RTIMULib/RTIMULib RTIMULib)

add_executable(imuPublisher src/imuPublisher.cpp)
target_link_libraries(imuPublisher ${catkin_LIBRARIES}
	RTIMULib
)
