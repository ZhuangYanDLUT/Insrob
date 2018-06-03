#ifndef LOCAL_MAP_OBSTACLE_CLASSIFICATION_H
#define LOCAL_MAP_OBSTACLE_CLASSIFICATION_H

#include "parameter.h"
void data_out(map<int,Obstacle_Feature>&Obs_feature,vector<vector<PointData> >&Linepoint);
void Obs_Classification(map<int,Obstacle_Feature>&Obs_feature,vector<map<float,PointData> >&_pointcloud);
#endif
