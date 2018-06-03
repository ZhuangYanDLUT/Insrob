#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include "server/vfileoperation.h"
#include "server/VSocket.h"
#include "server/filename.h"
#include <cstdlib>
#include <iostream>
#include <string>
using namespace std;

VSocket *pmonitor = NULL;//socket通信的全局变量
bool is_send_pc = true;
bool is_send_pos = true;
bool is_connect_on = false;
bool is_send_data = false;

char *pbufferPC = NULL;
char bufferPos[400];
std::vector<VPointI> cloudI;
std::vector<VPoseStamped> tmppose;
//pcl::PointCloud<pcl::PointXYZI>::Ptr airpoints(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr airpoints(new pcl::PointCloud<pcl::PointXYZINormal>());
float current_x = 0.0, current_y=0.0, current_z=0.0;

int socket_cmd_ret = 0;
static ros::ServiceClient map_name_send;

void start_map_server(){
        server::filename srv;
        srv.request.cmd = socket_cmd_ret;				
        srv.request.filename = pmonitor->priorfilename;
		/*		    
        string surf_file =  pmonitor->priorfilename + "_surf.map";
        string corner_file = pmonitor->priorfilename + "_corner.map";
				
        cout << "mapping : " << surf_file << " --- " <<  corner_file  << endl;
        //*/
        while(1){
            if(map_name_send.call(srv)){
                    cout << "反馈为: " << srv.response.feedback << endl;
                    break;
             }
        }
}

//发送点云重复的代码
void point_repeat(const sensor_msgs::PointCloud2ConstPtr& pointsData){
	int len = 0;
	
	//将ROS点云消息 转换为PCL点云格式
	airpoints->clear();
	pcl::fromROSMsg(*pointsData, *airpoints);
	
	//计算滤除无效点后的数量
	int cnt = 0;
	VPointI point1;
	for (size_t i = 0; i < airpoints->points.size(); ++i){
	    point1.x = airpoints->points[i].z;  //注意，这里带着坐标轴转换
		point1.y = airpoints->points[i].x;
		point1.z = airpoints->points[i].y;
	
		//过滤掉1.5m以内的点
		if(((point1.x-current_x)*(point1.x-current_x) + (point1.y-current_y)*(point1.y-current_y) + (point1.z-current_z)*(point1.z-current_z))  < 2.25)
		{ continue; }
	
		cnt++;
	}
	
 	//整理数据包头
	CPackage pack;
	pack.msgType = CMD_POINT_CLOUD;
	pack.msgLen = cnt;
	
	//int len = 0;
	char* str = pbufferPC;
	memcpy(str,(char*)(&pack),sizeof(CPackage));
	len += sizeof(CPackage);
	str += sizeof(CPackage);
	
	//整理数据包内容
	VPointI point;
	//cout << "发送点云：" << airpoints->points.size() << endl;
	cloudI.reserve(cnt);
	
	for (size_t i = 0; i < airpoints->points.size(); ++i){
        point.x = airpoints->points[i].z;  //注意，这里带着坐标轴转换
        point.y = airpoints->points[i].x;
        point.z = airpoints->points[i].y;
		
		point.intensity = airpoints->points[i].curvature;  //curvature中存储着强度值
		
		if(((point.x-current_x)*(point.x-current_x) + (point.y-current_y)*(point.y-current_y) + (point.z-current_z)*(point.z-current_z))  < 2.25)
		{ continue; }
		
		memcpy(str,(char*)(&point),sizeof(VPointI));
		len += sizeof(VPointI);
		str += sizeof(VPointI);
		//save cloud gxt
		
		if(pmonitor->isPointSaveServer)
		{
			cloudI.push_back(point);
		}
 		//cout<<"x="<<point.x<<",y="<<point.y<<",z="<<point.z<<endl;
	}
	
	if(pmonitor->isPointSaveServer)
	{
		VFileOperation::fileSavePCxyzi(pmonitor->current_path + "/points.xyzi",cloudI);
		cloudI.clear();
	}
	
	if(!is_send_pc) return;
	pmonitor->SendPC( pbufferPC, len);
}

void mapping_pc_callback(const sensor_msgs::PointCloud2ConstPtr& pointsData)
{
	if(!is_send_data) return;
	point_repeat(pointsData);
}

void pose_repeat(const nav_msgs::Odometry::ConstPtr& poseData)
{
	//============= 先整理出位姿 ======================
	float px,py,pz;			//位置
	double x,y,z,w;				//姿态四元数
	double yaw, pitch, roll; 	//姿态欧拉角：单位弧度
	
	px = poseData->pose.pose.position.z;  //注意，这里带着坐标轴转换
	py = poseData->pose.pose.position.x;
	pz = poseData->pose.pose.position.y;
		
	x = poseData->pose.pose.orientation.z;  //注意，这里带着转换
	y = poseData->pose.pose.orientation.x;
	z = poseData->pose.pose.orientation.y;
	w = poseData->pose.pose.orientation.w;
		
   	tf::Matrix3x3(tf::Quaternion(x, y, z, w)).getRPY(roll, pitch, yaw);
		
	//暂存一下当前位置，过滤激光点使用
	current_x = px;
	current_y = py; 
	current_z = pz;
	
	
	//整理数据包头
	CPackage pack;
	pack.msgType = CMD_ROBOT_POSE;
	pack.msgLen = 1;
	
	//整理数据包内容
	int len = 0;
	char* str = bufferPos;
	memcpy(str,(char*)(&pack),sizeof(CPackage));
	len += sizeof(CPackage);
	str += sizeof(CPackage);

	VPoseStamped the_pose;
	the_pose.timestamp = poseData->header.stamp.toSec();
	the_pose.x = px;
	the_pose.y = py;
	the_pose.z = pz;
	the_pose.i = x;
	the_pose.j = y;
	the_pose.k = z;
	the_pose.w = w;
	the_pose.yaw = yaw * 180.0 / 3.1415926;
	the_pose.pitch = pitch * 180.0 / 3.1415926;
	the_pose.roll = roll * 180.0 / 3.1415926;
		
	memcpy(str,(char*)(&the_pose),sizeof(VPoseStamped));
	len += sizeof(VPoseStamped);
	str += sizeof(VPoseStamped);
	if(pmonitor->isPoseSaveServer)
	{
		tmppose.reserve(1);
		tmppose.push_back(the_pose);
		VFileOperation::fileSavePoseStamped(pmonitor->current_path + "/poses.txt",tmppose);
		tmppose.clear();
	}
	
	if(!is_send_pos) return;
	pmonitor->SendPos( bufferPos, len);	
}

void mapping_pose_callback(const nav_msgs::Odometry::ConstPtr& poseData){
	if(!is_send_data) return;
	pose_repeat(poseData);	
}

//运行shell命令
int run_shell(const char* cmd)
{
	FILE *fp = popen(cmd, "r");
	if(fp == NULL)
	{
		return -1;
	}
	//char line[10240];
	//while(fgets(line, 10240, fp) != NULL) 
	//{
	//	cout << line;
	//}
	pclose(fp);
	return 0;
}


int main(int argc,char** argv)
{
	ros::init(argc,argv,"server");
	ros::NodeHandle n;

	if(pmonitor == NULL)
		pmonitor = new VSocket;
	
	pbufferPC = new char[2000000];

	ros::Subscriber mapping_pc_sub = n.subscribe("/velodyne_cloud_registered",2,mapping_pc_callback);
	ros::Subscriber mapping_pose_sub = n.subscribe("/integrated_to_init",100,mapping_pose_callback);

    map_name_send = n.serviceClient<server::filename>("mapname");           
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		if(!is_connect_on)
		{
			pmonitor->initSocket();
			is_connect_on = true;
			cout << "建立连接！" << endl;
		}

		if(is_connect_on)
		{
			int ret = pmonitor->Accept();
			switch(ret)
			{
			case 5: //结束运行算法（并且断开链接）
			{
				run_shell("/home/robot/robot_ws/kill.sh");
				pmonitor->close_current_sock();
				is_connect_on = false;
				is_send_data =false;
				cout << "断开当前连接！" << endl;
				break;
			}
			case 6:
			{
				is_send_data = true;
				is_send_pos = pmonitor->isPoseTransBack;
				is_send_pc = pmonitor->isPointTransBack;
				run_shell("/home/robot/robot_ws/run-mapping-rosbag.sh");
				cout << "开始运行mapping！" << endl;
				
				socket_cmd_ret = 1;
				start_map_server();
				break;
			} 
			case 7:
			{
				run_shell("/home/robot/robot_ws/kill.sh");
				is_send_data = false;
				cout << "结束mapping！" << endl;
				break;
			}
			case 8:
			{
				//注意，这之前在pmonitor有传递先验地图文件操作
				//run_shell("/home/robot/robot_ws/run-locate.sh");
				
				is_send_data =true;
				is_send_pos = true;
				is_send_pc = true;
				run_shell("/home/robot/robot_ws/run-mapping-rosbag.sh");

				cout << "开始运行locating！" << endl;
				
				socket_cmd_ret = 2;
				start_map_server();
				break;
			}
			case 9:
			{
				run_shell("/home/robot/robot_ws/kill.sh");
				is_send_data =false;
				cout << "结束locating！" << endl;
			    break;
			} 
			default:
				break;	
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	delete []pbufferPC;
	//ROS_INFO("Ready to calculation.");

 	return 0;
}
