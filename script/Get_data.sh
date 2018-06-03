workspace_name="Insrob_ws"

lddate=`date +%Y_%m_%d_%H_%M`
dirname=${lddate}

mkdir -p ~/Insrob_exe/log/${dirname}/

cd ~/$workspace_name/

source ~/$workspace_name/devel/setup.bash

./kill.sh


sudo chmod 777 /dev/ttyUSB1

sleep 2

sudo chmod 777 /dev/ttyUSB0

sleep 2

roslaunch rslidar_pointcloud rs_lidar_16.launch &

sleep 8

rosrun Base_control app_Controlservice &

sleep 2

rosrun Insrob_server app_Insrob_server & 

sleep 2 

rosrun gps_pub gps_pub_node &





