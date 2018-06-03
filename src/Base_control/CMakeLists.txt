cmake_minimum_required(VERSION 2.8.3)
project(Base_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation  #自定义
)

add_message_files(FILES 
		  Insrobpose.msg
      Motorfeedback.msg
		)
#----------需更新处---------------
add_service_files(   
	FILES   
	ControlCmd.srv  #自定义
)
#---------------------------------
 #用户自定义
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

#生成头文件
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Base_control
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/   #头文件必须添加这句申明
)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/ PROJECT_SOURCES) #.cpp必须添加这句申明（UART.cpp）


#---------------------需更新处------------------------
add_executable(app_Controlservice src/Service.cpp src/UART.cpp)
target_link_libraries(app_Controlservice ${catkin_LIBRARIES} rt)
add_dependencies(app_Controlservice Base_control_gencpp)

add_executable(Client1 src/Client1.cpp)
target_link_libraries(Client1 ${catkin_LIBRARIES})
add_dependencies(Client1 Base_control_gencpp)

#------------------------------------------------------

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
