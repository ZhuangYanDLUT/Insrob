#ifndef LOCAL_MAP_OBSTACLE_VIEW_H
#define LOCAL_MAP_OBSTACLE_VIEW_H

#include "parameter.h"
#include <fstream>
#include <cv_bridge/cv_bridge.h> 
#include <opencv2/video/tracking.hpp>  
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/core/core.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/thread/mutex.hpp>
using namespace cv; 
void* Show_object(void*);
void Draw_object(My_Pose pose,map<int,Obstacle_Feature>&hisObs_feature);
#endif
