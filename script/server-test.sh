workspace_name="Insrob_ws"
basepath=$(cd `dirname $0`;pwd)

lddate=`date +%Y_%m_%d_%H_%M`
dirname=${lddate}

Base_controllog=${lddate}'Base_control.txt'
app_Insrob_servelog=${lddate}'app_Insrob_servelog.txt'

./kill.sh

cd ..

mkdir -p ~/Insrob_exe/log/${dirnasme}/

cd ~/${workspace_name}

source devel/setup.bash

sudo chmod 777 /dev/ttyUSB1

sleep 2

sudo chmod 777 /dev/ttyUSB0

sleep 2

roscore &

sleep 2

roslaunch rslidar_pointcloud rs_lidar_16.launch &

sleep 2

roslaunch slam_algorithm algorithm_hand.launch &

sleep 8

rosrun transform_data transform_data &

sleep 2

rosrun Insrob_server app_Insrob_server &

sleep 2

rosrun Insrob_server speed_transform &

sleep 2

rosrun Base_control app_Controlservice &

sleep 2

rosrun xsens_imu xsens_imu &






