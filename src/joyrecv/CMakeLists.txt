cmake_minimum_required(VERSION 2.8.3)
project(joyrecv)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set output binary directory
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set include files
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)

#set source files
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/src PROJECT_SOURCES)

add_executable(app_Joystick_controller ${PROJECT_SOURCES})

## Specify libraries to link a library or executable target against
target_link_libraries(app_Joystick_controller ${catkin_LIBRARIES})
