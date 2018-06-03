#ifndef LOCAL_MAP_OBSTACLE_H
#define LOCAL_MAP_OBSTACLE_H

#include <vector>
#include <iostream>

#include "parameter.h"
#include "obstacle_segmentation.h"
#include "obstacle_kaman.h"
#include "obstacle_hungarian.h"
#include "obstacle_distance.h"
#include "obstacle_view.h"
#include "obstacle_classification.h"
//using namespace std;
const float time_frequency=10;//消息频率10hz

void rec_obstacle(vector<map<float,PointData> >&_pointcloud,std::vector<GridData>&mapData,Map_info mapinfo,My_Pose pose);
void Obs_match(My_Pose pose,map<int,Obstacle_Feature>&Obs_feature,map<int,Obstacle_Feature>&hisObs_feature);

#endif 
