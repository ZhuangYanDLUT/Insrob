#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include <cmath>
#include <stdlib.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>

ros::Subscriber get_pose;
ros::Subscriber get_points;
ros::Publisher pub_pose;
ros::Publisher pub_points;
ros::Publisher marker_pub ;
pcl::PointCloud<pcl::PointXYZINormal>::Ptr airpoints(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZI> airpoints3;

bool isoutfile = false;
std::ofstream outfile;

float current_x = 0.0, current_y=0.0, current_z=0.0;

void get_poseCallback(const nav_msgs::Odometry::ConstPtr& poseData)
{

	// std::cout<<"###########################before pose---------------"<<std::endl;
 //  	std::cout<<"poseData->pose.pose.position.x "<<std::endl;
 //  	std::cout<<poseData->pose.pose.position.x<<" "<<poseData->pose.pose.position.y<<" "<<poseData->pose.pose.position.z<<std::endl;
 //  	std::cout<<"yaw roll pitch "<<std::endl;
 //  	double yaw1,roll1,pitch1;
 //  	tf::Matrix3x3(tf::Quaternion(poseData->pose.pose.orientation.x, poseData->pose.pose.orientation.y, poseData->pose.pose.orientation.z, poseData->pose.pose.orientation.w)).getRPY(roll1, pitch1, yaw1);
 //  	std::cout<<yaw1*180/3.14<<" "<<roll1*180/3.14<<" "<<pitch1*180/3.14<<std::endl; 



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

  // std::cout<<px<<" "<<py<<" "<<pz<<" "<<yaw*180/3.14<<std::endl; 

  outfile<<px<<" "<<py<<" "<<pz<<" "<<yaw<<std::endl; 

  // std::cout<<"-----------------------------------pose---------------"<<std::endl;
  // std::cout<<"px py pz "<<std::endl;
  // std::cout<<px<<" "<<py<<" "<<pz<<std::endl;
  // std::cout<<"yaw roll pitch "<<std::endl;
  // std::cout<<yaw*180/3.14<<" "<<roll*180/3.14<<" "<<pitch*180/3.14<<std::endl; 

	static tf::TransformBroadcaster tfBroadcaster;
    static tf::Transform transform;
    tf::Quaternion q;

    transform.setOrigin(tf::Vector3(px, py, pz));//相对地面xyz
	
	ros::Time time = poseData->header.stamp;
	q.setRPY(roll, pitch, yaw);
	transform.setRotation(q);//待调整
	tfBroadcaster.sendTransform(tf::StampedTransform(transform,time,"/camera_init", "/rslidar"));

		

	nav_msgs::Odometry pubpose;

	pubpose.header.stamp = poseData->header.stamp;

	pubpose.header.frame_id = poseData->header.frame_id;

	pubpose.child_frame_id = poseData->child_frame_id;	

	pubpose.pose.pose.position.x = px;
	pubpose.pose.pose.position.y = py;
    pubpose.pose.pose.position.z = pz + 0.8;
	
	pubpose.pose.pose.orientation.x = x;
	pubpose.pose.pose.orientation.y = y;
	pubpose.pose.pose.orientation.z = z;
	pubpose.pose.pose.orientation.w = w;

	current_x = px;
	current_y = py;
	current_z = pz;
	
	pub_pose.publish(pubpose);
}


void get_pointsCallback(const sensor_msgs::PointCloud2ConstPtr& pointsData)
{
	airpoints->clear();
	pcl::fromROSMsg(*pointsData, *airpoints);

	// std::cout<<"PointCloud2ConstPtr header ----------------------------------------------"<<std::endl;
	// std::cout<<pointsData->header.seq<<std::endl;
	// std::cout<<pointsData->header.stamp<<std::endl;   
	// std::cout<<pointsData->header.frame_id<<std::endl; 

	for (size_t i = 0; i < airpoints->points.size(); ++i)
	{

	   pcl::PointXYZI p1;

	   p1.x =airpoints->points[i].z;
	   p1.y =airpoints->points[i].x;
	   p1.z =airpoints->points[i].y;
	   p1.intensity = airpoints->points[i].intensity;

	   if(((p1.x-current_x)*(p1.x-current_x) + (p1.y-current_y)*(p1.y-current_y) + (p1.z-current_z)*(p1.z-current_z))  < 2.25)
		{ continue; }

	   airpoints3.points.push_back(p1);

	}


	sensor_msgs::PointCloud2 airpoints2;
    pcl::toROSMsg(airpoints3, airpoints2);

	airpoints3.points.clear();

    airpoints2.header.frame_id = pointsData->header.frame_id;
    airpoints2.header.stamp = pointsData->header.stamp;

	/*std::cout<<"airpoints2 header ----------------------------------------------"<<std::endl;
	std::cout<<airpoints2.header.seq<<std::endl;
	std::cout<<airpoints2.header.stamp<<std::endl;   
	std::cout<<airpoints2.header.frame_id<<std::endl;*/

 	pub_points.publish(airpoints2);
}


int main(int argc, char** argv)
{
	if(isoutfile)
	{
		outfile.open("/home/robot/2017-11-25-15-37-55.txt",std::ios::out|std::ios::app);
	}

  

  ros::init(argc, argv, "transform_data");
  ros::NodeHandle n;


  get_pose = n.subscribe<nav_msgs::Odometry>("/integrated_to_init",2, get_poseCallback);
  get_points = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered",2,get_pointsCallback);

  pub_pose = n.advertise<nav_msgs::Odometry>("/air_pose", 2);
  pub_points = n.advertise<sensor_msgs::PointCloud2>("/air_points", 2);
  marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

  while (ros::ok())
  {
      ros::spinOnce();
  }

  outfile.close();
}



// using	std::string;
// using	std::vector;
// using namespace std;
// using namespace Eigen;

// Eigen::Matrix4f tf_before=Eigen::Matrix4f::Identity();
// Eigen::Matrix4f init_guess=Eigen::Matrix4f::Identity();
// Eigen::Matrix4f tf_y=Eigen::Matrix4f::Identity();
// Eigen::Matrix4f tf_z=Eigen::Matrix4f::Identity();

// ros::Subscriber get_pose;
// ros::Subscriber get_points;
// ros::Publisher pub_pose;
// ros::Publisher pub_points;
// ros::Publisher marker_pub ;
// pcl::PointCloud<pcl::PointXYZ> airpose;
// pcl::PointCloud<pcl::PointXYZINormal>::Ptr airpoints(new pcl::PointCloud<pcl::PointXYZINormal>());
// pcl::PointCloud<pcl::PointXYZI> airpoints3;

// typedef pcl::PointXYZ PointType;
// typedef pcl::PointCloud<PointType> PCLPointCloud;


// pcl::PointXYZ po;

// // Set the frame ID and timestamp.  See the TF tutorials for information on these.


// // Set the namespace and id for this marker.  This serves to create a unique ID
// // Any marker sent with the same namespace and id will overwrite the old one



// void get_poseCallback(const nav_msgs::Odometry::ConstPtr& poseData)
//  {
//    pcl::PointXYZ p1;

//    p1.x =poseData->pose.pose.position.z;
//    p1.y =poseData->pose.pose.position.x;
//    p1.z =poseData->pose.pose.position.y;
//    po.x =p1.x;
//    po.y =p1.y;
//    po.z =p1.z;
//    airpose.points.push_back(p1);

//    //pcl::transformPointCloud(airpose,airpose,tf_before);
//    //pcl::transformPointCloud(airpose,airpose,init_guess);

//    sensor_msgs::PointCloud2 airpose2;
//    pcl::toROSMsg(airpose, airpose2);
//    airpose2.header.frame_id = "/camera_init";
//    pub_pose.publish(airpose2);
//    airpose.clear();
   
   
//    {
// 	//定义旋转
// 	Eigen::AngleAxisf rotation1 (1.0*M_PI/2.0, Eigen::Vector3f::UnitY ());
// 	Eigen::Quaternionf q1(rotation1);
// 	Eigen::AngleAxisf rotation2 (1.0*M_PI/2.0, Eigen::Vector3f::UnitX ());
// 	Eigen::Quaternionf q2(rotation2);
// 	Eigen::Quaternionf q12 = q2*q1;

// 	//transform position
// 	Eigen::Vector3f p0(poseData->pose.pose.position.x, poseData->pose.pose.position.y, poseData->pose.pose.position.z);
// 	Eigen::Vector3f p_finall = q12._transformVector(p0);
	
// 	//transform orientation
//    	Eigen::Quaternionf q0 = Eigen::Quaternionf(poseData->pose.pose.orientation.w,
// 																poseData->pose.pose.orientation.x,
// 																poseData->pose.pose.orientation.y,
// 																poseData->pose.pose.orientation.z);
// 	Eigen::Quaternionf q_finall = q12*q0;
	
// 	double x = q_finall.x();
// 	double y = q_finall.y();
// 	double z = q_finall.z();
// 	double w = q_finall.w();

// 	//由四元数得到欧拉角
// 	double yaw,pitch,roll;
//    	tf::Matrix3x3(tf::Quaternion(z, -x, -y, w)).getRPY(roll, pitch, yaw);
	
// 	cout<<"--------------"<<endl;
// 	cout<<p0<<endl;
// 	cout<<p_finall<<endl;
// 	cout<<"("<<p1.x<<", "<<p1.y<<", "<<p1.z<<")"<<endl;
// 	std::cout<<"Euler, yaw:"<<yaw* 180.0 / 3.1416<<", pitch:"<<pitch* 180.0 / 3.1416<<", roll:"<<roll* 180.0 / 3.1416<<std::endl;
	
// 	}
	

//    visualization_msgs::Marker marker;
//    marker.header.stamp = ros::Time::now();

//    marker.ns = "basic_shapes";
//    marker.id = 0;
//    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
//    marker.mesh_resource = "package://test_record_data/meshes/quadrotor/quadrotor_base.dae";
//    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
//    //marker.type = shape;

//    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
//    marker.action = visualization_msgs::Marker::ADD;


//    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
//    marker.pose.position.x = poseData->pose.pose.position.z;
//    marker.pose.position.y = poseData->pose.pose.position.x;
//    marker.pose.position.z = poseData->pose.pose.position.y;
//    marker.pose.orientation.x = 0.0;
//    marker.pose.orientation.y = 0.0;
//    marker.pose.orientation.z = 0.0;
//    marker.pose.orientation.w = 1.0;

//    // Set the scale of the marker -- 1x1x1 here means 1m on a side
//    marker.scale.x = 2;
//    marker.scale.y = 2;
//    marker.scale.z = 2;

//    // Set the color -- be sure to set alpha to something non-zero!
//    marker.color.r = 1.0f;
//    marker.color.g = 1.0f;
//    marker.color.b = 1.0f;
//    marker.color.a = 1.0;

//   // marker.lifetime = ros::Duration();
//    marker.lifetime = ros::Duration();
//    // Publish the marker
//    marker.header.frame_id = "/camera_init";
//    marker_pub.publish(marker);

//  }

// void get_pointsCallback(const sensor_msgs::PointCloud2ConstPtr& pointsData)
// {
//   airpoints->clear();
//   pcl::fromROSMsg(*pointsData, *airpoints);
//   for (size_t i = 0; i < airpoints->points.size(); i++) {
//      pcl::PointXYZI p2;
//      p2.x= airpoints->points[i].z;
//      p2.y= airpoints->points[i].x;
//      p2.z= airpoints->points[i].y + 0.8;
//      p2.intensity=airpoints->points[i].intensity;
//      if (sqrt((p2.x-po.x)*(p2.x-po.x)+(p2.y-po.y)*(p2.y-po.y)+(p2.z-po.z)*(p2.z-po.z))>1) {
//       airpoints3.points.push_back(p2);
//      }

//   }
//   //pcl::transformPointCloud(*airpoints,*airpoints,tf_before);
//  // pcl::transformPointCloud(airpoints3,airpoints3,init_guess);

//   sensor_msgs::PointCloud2 airpoints2;
//   pcl::toROSMsg(airpoints3, airpoints2);
//   airpoints2.header.frame_id = "/camera_init";
//   pub_points.publish(airpoints2);
//   airpoints3.points.clear();
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "transform_data");
//   ros::NodeHandle n;


//   tf_before(0,0)=1;tf_before(0,1)=0;tf_before(0,2)=0;tf_before(0,3)=0;
//   tf_before(1,0)=0;tf_before(1,1)=0;tf_before(1,2)=-1;tf_before(1,3)=0;
//   tf_before(2,0)=0;tf_before(2,1)=1;tf_before(2,2)=0;tf_before(2,3)=0;
//   tf_before(3,0)=0;tf_before(3,1)=0;tf_before(3,2)=0;tf_before(3,3)=1;


//   Eigen::AngleAxisf init_rotation (0, Eigen::Vector3f::UnitY ());
//   Eigen::Translation3f init_translation (0, 0, 0);
//   init_guess = (init_translation * init_rotation).matrix ();

//   //////////////////////////////////////////////////////////////////////



//   get_pose = n.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 1000, get_poseCallback);
//   get_points = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered",2,get_pointsCallback);
//   pub_pose = n.advertise<sensor_msgs::PointCloud2>("/air_pose", 2);
//   pub_points = n.advertise<sensor_msgs::PointCloud2>("/air_points", 2);
//   marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
//   while (ros::ok())
//   {
//       ros::spinOnce();
//   }
// }

