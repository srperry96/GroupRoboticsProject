cmake_minimum_required(VERSION 2.8.3)
project(lineUpTeddyPackage)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS}
)

add_executable(teddyGripController src/teddyGripController.cpp)
target_link_libraries(teddyGripController ${catkin_LIBRARIES}
)
