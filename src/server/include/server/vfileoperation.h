#ifndef VFILEOPERATION_H
#define VFILEOPERATION_H

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include "server/vpose.h"
#include "server/vpoint.h"
using namespace std;

class VFileOperation 
{
public:
    VFileOperation();
    ~VFileOperation();

public:
    static void fileSavePCxyzi(const string path, std::vector<VPointI>&cloud);
    
    static void fileSavePose(const string path, std::vector<VPose>&pose);
	static void fileSavePoseStamped(const string path, std::vector<VPoseStamped>&pose);

    static void saveInfoTXT(const string path, string time, bool isSaveNavMap);

};
#endif //VFILEOPERATION_H
