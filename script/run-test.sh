workspace_name="Insrob_ws"
basepath=$(cd `dirname $0`;pwd)

lddate=`date +%Y_%m_%d_%H_%M`
dirname=${lddate}

transform_datalog=${lddate}'transform_data.txt'
local_maplog=${lddate}'local_map.txt'
path_plannerlog=${lddate}'path_planner.txt'

./kill.sh

cd ..

mkdir -p ~/Insrob_exe/log/${dirname}/

cd ~/${workspace_name}

source devel/setup.bash

sudo chmod 777 /dev/ttyUSB1

rosrun Base_control app_Controlservice &



sleep 2

roslaunch rslidar_pointcloud rs_lidar_16.launch &

sleep 2

roslaunch slam_algorithm algorithm_hand.launch   &

sleep 6

rosservice call /mapname 2 Insrob &

sleep 1

rosrun transform_data transform_data 1>../Insrob_exe/log/${dirname}/${transform_datalog}&

sleep 1

rosrun local_map app_local_map 1>../Insrob_exe/log/${dirname}/${local_maplog}&

sleep 1

rosrun path_planner app_path_planner 1>../Insrob_exe/log/${dirname}/${path_plannerlog} &






