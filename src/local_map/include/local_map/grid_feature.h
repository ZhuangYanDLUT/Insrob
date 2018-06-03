#ifndef LOCAL_MAP_GRIDFEATURE_H
#define LOCAL_MAP_GRIDFEATURE_H
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

#include "parameter.h"
#include "point.h"

void rec_gridfeature(vector<map<float,PointData> >&pointcloud,Map_info mapinfo,std::vector<GridData> &mapData);
#endif 
