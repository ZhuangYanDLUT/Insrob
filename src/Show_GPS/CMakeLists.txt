cmake_minimum_required(VERSION 2.8.3)
project(Show_GPS)
set(CMAKE_BUILD_TYPE RELEASE)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  message_generation
)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
include_directories(${catkin_INCLUDE_DIRS})

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/ PROJECT_SOURCES)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs geometry_msgs nav_msgs
)


add_executable(app_show_pose ${PROJECT_SOURCES})
target_link_libraries(app_show_pose ${catkin_LIBRARIES})
add_dependencies(app_show_pose app_show_pose_gencpp)


