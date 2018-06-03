workspace_name="Insrob_ws"

lddate=`date +%Y_%m_%d_%H_%M`
dirname=${lddate}

joylog=${lddate}'joylog.txt'
joyrecvlog=${lddate}'joyrecvlog.txt'
Base_controllog=${lddate}'Base_control.txt'


mkdir -p ~/Insrob_exe/log/${dirname}/

source ~/$workspace_name/devel/setup.bash

./kill.sh

sudo chmod 777 /dev/ttyUSB0

sleep 2

rosrun Base_control app_Controlservice &

sleep 8

rosrun joy joy_node 1>../Insrob_exe/log/${dirname}/${local_maplog}&

rosrun joyrecv app_Joystick_controller 1>../Insrob_exe/log/${dirname}/${path_plannerlog} &
