#ifndef LOCAL_MAP_OBSTACLE_SEGMENTATION_H
#define LOCAL_MAP_OBSTACLE_SEGMENTATION_H

#include "parameter.h"
const int Min_obsnum=4;//最少栅格数
const int Max_obsnum=25;//最多栅格数
void Obs_Segmentation(std::vector<GridData>& mapData,Map_info mapinfo,map<int,Obstacle_Feature>& Obs_feature);
#endif 
