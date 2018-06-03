cmake_minimum_required(VERSION 2.8.3)
project(gps_pub)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   imu_gps.msg
  )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
)

	
## Build serial_port

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/ PROJECT_SOURCES)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/ ${catkin_INCLUDE_DIRS})


add_executable(gps_pub_node ${PROJECT_SOURCES})

add_dependencies(gps_pub_node gps_pub_generate_messages_cpp)

target_link_libraries(gps_pub_node ${catkin_LIBRARIES})
