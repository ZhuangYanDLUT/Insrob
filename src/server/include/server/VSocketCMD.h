#ifndef VSOCKETCMD_H
#define VSOCKETCMD_H
using namespace std;
//控制命令
enum CmdType
{
    CMD_POINT_CLOUD = 1,//通知服务器端发送点云数据
    CMD_ROBOT_POSE,//通知服务器端发送位姿数据
    CMD_STOP_END,//通知服务器端,结束socket
    CMD_START_ALGO,//开始算法
    CMD_STOP_ALGO,//结束算法
    CMD_START_LOCATION,//开始定位算法
    CMD_STOP_LOCATION//
};

//接收的数据包头的结构
struct CPackage {
    CmdType msgType;
    unsigned int msgLen;
};

struct StartPackage {
    char time[32];
    bool isPointSaveServer;
    bool isPoseSaveServer;
    bool isSaveNavMap;
    bool isPointTransBack;
    bool isPoseTransBack;
};

struct LocatePackage {
    char ScanID[32];
    bool isPointTransBack;
    bool isPoseTransBack;

};


#endif // VSOCKETCMD_H
