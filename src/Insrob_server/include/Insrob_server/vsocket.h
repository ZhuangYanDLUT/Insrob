#ifndef VSOCKET_H
#define VSOCKET_H
#include "Insrob_server/global.h"
#include "Insrob_server/vpoint.h"
using namespace std;

#define BUF_SIZE 128

class VSocket
{
public:
    VSocket();
    ~VSocket();
public:
    bool Start();//通信初始化
    int Accept();//从socket的accept函数
    CPackage RecvData();//接收数据
    bool SendPos(char *pointpose,const int length);//发送位姿数据
    bool SendPC(char *pointcloud,const int length);//发送点云数据
    void Terminate();//通信终止
    void close_current_sock();

    bool isPointSaveServer;
    //bool isXYZ;
    bool isPoseSaveServer;
    bool isSaveNavMap;
    bool isPointTransBack;
    bool isPoseTransBack;
    bool isPointTransSparse;
    string current_path;
    string priorfilename;
public:
    bool m_bissuccessInit;//标识初始化是否成功
    bool m_is_locate;//标志是否是定位模式，用于屏蔽Accept函数的接收
private:
    int recv_large(int len, char* buffer);
    void recv_prior_map_file(int len);//接收客户端发来的先验地图文件
//  bool ForbidenBlock();//防阻塞
    //void createDir(const char* path);
private:
    char *buffer;
    int serv_sock;//soceket for listenning 
    int clnt_sock;//socket for transmission
//  int m_iLen;//数据长度
};

#endif // VSOCKET_H
