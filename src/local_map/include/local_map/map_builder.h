#ifndef LOCAL_MAP_MAP_BUILDER_H
#define LOCAL_MAP_MAP_BUILDER_H

#include <vector>
#include <math.h>
#include <set>
#include <sstream>
#include <fstream>
#include <string>
#include <stdlib.h>  
#include <stdio.h>  

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/LaserScan.h>
#include <boost/thread/mutex.hpp>

#include "io.h"
#include "parameter.h"
#include "point.h"
#include "data_init.h"
#include "road_boundary.h"
#include "grid_feature.h"
#include "obstacle.h"

namespace local_map 
{
	class MapBuilder
	{

	public:
		/*构造&析构函数*/
		MapBuilder(unsigned short width, unsigned short height, double resolution);
		~MapBuilder();

	public:
		
		/*程序入口:更新地图，在main.cpp激光回调函数中调用*/
		void updateMap(sensor_msgs::PointCloud2 &pointClouds_msg);
		/*程序入口:更新位姿，在main.cpp激光回调函数中调用*/
		void setPoseAndHeading(float x, float y, float z, float angle);
		/*程序出口:输出地图，在main.cpp激光回调函数中调用*/
		nav_msgs::OccupancyGrid getMap();

	private:

		std::vector<GridData> mapData;			//地图中每个栅格的特征，其他函数对此变量进行处理
		nav_msgs::OccupancyGrid map_;			//输出的地图变量
		std::vector<GridData> history_mapData;	//栅格的特征
		/*根据栅格特征mapData，生成新的栅格地图map_*/
		void generate_map();

	private:
		My_Pose _pose;//机器人位姿
		Map_info mapinfo;//地图参数
		
	};
}

#endif  // LOCAL_MAP_MAP_BUILDER_H
