#ifndef LOCAL_MAP_DATATRANSITION_H
#define LOCAL_MAP_DATATRANSITION_H
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <math.h>

#include "parameter.h"
//#include "point.h"

void rec_initdata(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,vector<map<float,PointData> >&_pointcloud,My_Pose _pose);
void rec_oldmap(std::vector<GridData>& mapData,std::vector<GridData>& history_mapData,Map_info& mapinfo,My_Pose _pose);
void rec_savemap(std::vector<GridData>& mapData,std::vector<GridData>& history_mapData);
#endif 
