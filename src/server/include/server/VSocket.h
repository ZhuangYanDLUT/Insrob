#ifndef VSOCKET_H_
#define VSOCKET_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>
#include <netdb.h>

#include "server/vpoint.h"
#include "server/vpose.h"
#include "server/VSocketCMD.h"
#include "vfileoperation.h"
using namespace std;

#define BUF_SIZE 128


class VSocket{
public:
	VSocket();
	~VSocket();
public:
	bool Start();//通信初始化
	void initSocket();//从socket的accept函数
	int Accept();//接收数据
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
	string current_path;
	
	string priorfilename;
private:
	int recv_large(int len, char* buffer);
	void recv_prior_map_file(int len);//接收客户端发来的先验地图文件
//	bool ForbidenBlock();//防阻塞
	//void createDir(const char* path);
public:
	bool m_bissuccessInit;//标识初始化是否成功
	bool m_is_locate;//标志是否是定位模式，用于屏蔽Accept函数的接收
private:
	char *buffer;
	int serv_sock;//主soceket
	int clnt_sock;//从socket
//	int m_iLen;//数据长度
};

#endif
