cmake_minimum_required(VERSION 2.8.3)
project(blobDetection)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	cv_bridge
	image_transport
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

add_executable(blobDetect src/blobDetect.cpp)
target_link_libraries(blobDetect ${catkin_LIBRARIES}
${OpenCV_LIBRARIES})
