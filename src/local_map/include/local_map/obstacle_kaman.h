#ifndef LOCAL_MAP_OBSTACLE_KAMAN_H
#define LOCAL_MAP_OBSTACLE_KAMAN_H

#include "parameter.h"
#include <cv_bridge/cv_bridge.h> 
#include <opencv2/video/tracking.hpp>  
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/core/core.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv; 
const int stateNum=4;       //状态值4×1向量(x,y,△x,△y)  
const int measureNum=2;     //测量值2×1向量(x,y)  
const float Deal_r=7.5;		//处理半径  

void ClearKaman();
void PredictPose(My_Pose pose,float r,vector<My_Person>&predict_pose);
void UpdataPose(vector<My_Person>&measure_pose);
#endif
