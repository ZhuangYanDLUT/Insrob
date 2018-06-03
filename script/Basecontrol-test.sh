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

sleep 2

roscore &

sleep 4


rosrun Insrob_server app_Insrob_server &

sleep 4

sudo chmod 777 /dev/ttyUSB0

sleep 4

rosrun Base_control app_Controlservice 1>/home/robot/${Base_controllog} &


