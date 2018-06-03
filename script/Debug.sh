workspace_name="Insrob_ws"
basepath=$(cd `dirname $0`;pwd)

lddate=`date +%Y_%m_%d_%H_%M`
dirname=${lddate}


slam_algorithmlog=${lddate}'slam_algorithm.txt'
transform_datalog=${lddate}'transform_data.txt'
local_maplog=${lddate}'local_map.txt'
path_plannerlog=${lddate}'path_planner.txt'

./kill.sh

cd ..

mkdir -p ~/Insrob_exe/log/${dirname}/

cd ~/${workspace_name}

source devel/setup.bash


roslaunch slam_algorithm algorithm_hand.launch 1>../Insrob_exe/log/${dirname}/${slam_algorithmlog} &

sleep 8

rosrun transform_data transform_data 1>../Insrob_exe/log/${dirname}/${transform_datalog} &

sleep 2

rosrun local_map app_local_map 1>../Insrob_exe/log/${dirname}/${local_maplog} &

sleep 2

rosrun path_planner app_path_planner 1>../Insrob_exe/log/${dirname}/${path_plannerlog} &

rviz





