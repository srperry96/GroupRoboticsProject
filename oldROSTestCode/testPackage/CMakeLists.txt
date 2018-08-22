cmake_minimum_required(VERSION 2.8.3)
project(testPackage)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS}
)

add_executable(testPublisher src/testPublisher.cpp)
target_link_libraries(testPublisher ${catkin_LIBRARIES})
