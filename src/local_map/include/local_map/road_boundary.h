#ifndef LOCAL_MAP_ROADBOUNDARY_H
#define LOCAL_MAP_ROADBOUNDARY_H
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

#include "parameter.h"
//#include "point.h"

void rec_roadboundary(vector<map<float,PointData> >&pointcloud, Map_info mapinfo, std::vector<GridData> &mapData);
#endif 
