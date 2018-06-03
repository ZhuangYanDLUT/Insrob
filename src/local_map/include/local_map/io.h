#ifndef LOCAL_MAP_IO_H
#define LOCAL_MAP_IO_H
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>

#include "parameter.h"
void outputData(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,int32_t Out_filenum);
#endif 
