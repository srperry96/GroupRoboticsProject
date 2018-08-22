cmake_minimum_required(VERSION 2.8.3)
project(ArduinoSPIComms)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS}
)

add_executable(arduinoSPIComms src/arduinoSPIComms.cpp
src/spiComms.cpp)
target_link_libraries(arduinoSPIComms ${catkin_LIBRARIES}
)
