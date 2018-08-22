cmake_minimum_required(VERSION 2.8.3)
project(MovementSubscriber)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
)

FIND_LIBRARY(pigpiod_if2_LIBRARIES pigpiod_if2 /usr/local
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS}
)


add_executable(movementSubscriber src/movementSubscriber.cpp
src/groundRobotMotorControl.cpp)
target_link_libraries(movementSubscriber ${catkin_LIBRARIES}
	pigpiod_if2
)
