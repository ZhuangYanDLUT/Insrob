#2017 10 18
#Isprobot 

workspace_name="Insrob_ws"
basepath=$(cd `dirname $0`;pwd)

echo "---->1. Create catkin space...\n"
#judge if workspace exists?
if [ -d ~/$workspace_name/src ]; then
	echo -n "workspace already exists, overlap ? [y/n]:"
	read res
	if [ $res == "n" ]; then
		exit 0
	else
		rm -fr ~/$workspace_name/
		rm -fr ~/Insrob_exe/
	fi
fi


mkdir -p ~/$workspace_name/src
mkdir -p ~/Insrob_exe/log


cd ~/workspace_name/src
catkin_init_workspace


cd ~/$workspace_name/

source ~/$workspace_name/devel/setup.bash

catkin_make

source ~/$workspace_name/devel/setup.bash

#check if success
if echo $ROS_PACKAGE_PATH |grep -a $workspace_name; then
	echo "Successfully create workspace!"
else
	echo "Create workspace failed!"
	exit 1
fi

#2) create pkgs and copy files
echo "---->2. Create pckgs and copy files...\n"
cd ~/$workspace_name/src


catkin_create_pkg Base_control

catkin_create_pkg rslidar
catkin_create_pkg rslidar_msgs
catkin_create_pkg rslidar_driver
catkin_create_pkg rslidar_pointcloud
catkin_create_pkg gps_pub

catkin_create_pkg Insrob_server
catkin_create_pkg joy
catkin_create_pkg joyrecv
catkin_create_pkg server

catkin_create_pkg Show_GPS
catkin_create_pkg path_planner
catkin_create_pkg local_map
catkin_create_pkg transform_data
catkin_create_pkg slam_algorithm

#3) ###########################################
cd $basepath

cp -rf ../src/Base_control/ ~/$workspace_name/src

cp -rf ../src/rslidar ~/$workspace_name/src
cp -rf ../src/rslidar_msgs ~/$workspace_name/src
cp -rf ../src/rslidar_driver ~/$workspace_name/src
cp -rf ../src/rslidar_pointcloud ~/$workspace_name/src
cp -rf ../src/gps_pub ~/$workspace_name/src




cp -rf ../src/Insrob_server/ ~/$workspace_name/src
cp -rf ../src/joy/ ~/$workspace_name/src
cp -rf ../src/joyrecv/ ~/$workspace_name/src
cp -rf ../src/server/ ~/$workspace_name/src


cp -rf ../src/Show_GPS/ ~/$workspace_name/src
cp -rf ../src/path_planner/ ~/$workspace_name/src
cp -rf ../src/local_map/ ~/$workspace_name/src
cp -rf ../src/transform_data/ ~/$workspace_name/src
cp -rf ../src/slam_algorithm/ ~/$workspace_name/src


cp -rf run-test.sh ~/$workspace_name/
cp -rf kill.sh ~/$workspace_name/
cp -rf build.sh ~/$workspace_name/
cp -rf joycontorl.sh ~/$workspace_name/
cp -rf Basecontrol-test.sh ~/$workspace_name/
cp -rf Get_data.sh ~/$workspace_name/
cp -rf Debug.sh ~/$workspace_name/
#4)#####################################################

#5)####################################################
echo "---->3. Catkin make...\n"
cd ~/$workspace_name/
source ~/$workspace_name/devel/setup.bash

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



#6)#######################################################

cd ~/$workspace_name/

mkdir -p ~/$workspace_name/map/

mkdir -p ~/$workspace_name/rviz/

cd ~/$workspace_name/src/path_planner/

mkdir -p ~/$workspace_name/src/path_planner/meshes/

cd $basepath/

cp -rf ../rviz/ ~/$workspace_name/rviz/

cp -rf ../meshes/ ~/$workspace_name/src/path_planner/meshes/









