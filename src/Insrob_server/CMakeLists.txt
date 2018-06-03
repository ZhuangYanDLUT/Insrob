cmake_minimum_required(VERSION 2.8.3)
project(Insrob_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_msgs
  visualization_msgs
  message_generation
)

find_package(PCL REQUIRED)

add_message_files(FILES 
	Speed.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_server
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs pcl_ros pcl_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/ PROJECT_SOURCES)

add_executable(app_Insrob_server src/client.cpp src/orient.cpp src/vsocket.cpp)
target_link_libraries(app_Insrob_server ${catkin_LIBRARIES})
add_dependencies(app_Insrob_server Insrob_server_gencpp)

add_executable(speed_transform src/speedtransform.cpp)
target_link_libraries(speed_transform ${catkin_LIBRARIES})
add_dependencies(speed_transform Insrob_server_gencpp)

