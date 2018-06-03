#ifndef LOCAL_MAP_OBSTACLE_DISTANCE_H
#define LOCAL_MAP_OBSTACLE_DISTANCE_H

#include "parameter.h"
#include <cmath>
void Cal_costMatrix(vector< vector<double> >&costMatrix,map<int,Obstacle_Feature>hisObs_feature,map<int,Obstacle_Feature>Obs_feature);
#endif
