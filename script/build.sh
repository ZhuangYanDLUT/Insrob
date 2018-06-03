workspace_name="Insrob_ws"
cd ~/$workspace_name/
source ~/$workspace_name/devel/setup.bash

catkin_make clean

source ~/$workspace_name/devel/setup.bash
catkin_make --pkg Base_control
source ~/$workspace_name/devel/setup.bash


cd ~/$workspace_name/src/rslidar_driver/
chmod 777 cfg/*
cd ~/$workspace_name/

cd ~/$workspace_name/src/rslidar_pointcloud/
chmod 777 cfg/*
cd ~/$workspace_name/


source ~/$workspace_name/devel/setup.bash
catkin_make --pkg rslidar_driver
source ~/$workspace_name/devel/setup.bash
catkin_make --pkg rslidar_pointcloud

catkin_make --pkg gps_pub
source ~/$workspace_name/devel/setup.bash
catkin_make --pkg Show_GPS

catkin_make --pkg Insrob_server

catkin_make --pkg joy

source ~/$workspace_name/devel/setup.bash
catkin_make --pkg joyrecv
source ~/$workspace_name/devel/setup.bash
catkin_make --pkg server
source ~/$workspace_name/devel/setup.bash


catkin_make --pkg local_map

source ~/$workspace_name/devel/setup.bash
catkin_make --pkg transform_data

catkin_make --pkg slam_algorithm

catkin_make --pkg path_planner
