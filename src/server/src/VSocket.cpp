#include "server/VSocket.h"
#include <sys/stat.h>
#include <sys/types.h>

using namespace std;
VSocket::VSocket()
{
	cout << "this is a server !" << endl;
	m_bissuccessInit = Start();
	if(false == m_bissuccessInit)	
		cout << "socket初始化失败..." << endl;	
	cout << "start to listen..." << endl;
	m_is_locate = false;
	//buffer = new char[2000000];
}

VSocket::~VSocket()
{
	//delete []buffer;
}

bool VSocket::Start()
{
	serv_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);//
	//将套接字和IP、端口绑定
	struct sockaddr_in serv_addr;
	memset(&serv_addr, 0, sizeof(serv_addr));  //每个字节都用0填充
	serv_addr.sin_family = AF_INET;  //使用IPv4地址
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);  //具体的IP地址
	serv_addr.sin_port = htons(2000);  //端口
	int ret = bind(serv_sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
	if(-1 == ret)
		return false;
	
	//进入监听状态，等待用户发起请求
	listen(serv_sock, 1);
	
	//initSocket();
	return true;
}

void VSocket::initSocket()
{
	//接收客户端请求
	struct sockaddr_in clnt_addr;
	socklen_t clnt_addr_size = sizeof(clnt_addr);
	clnt_sock = accept(serv_sock, (struct sockaddr*)&clnt_addr, &clnt_addr_size);
}

int VSocket::recv_large(int len, char* buffer)
{
    int count = 0;
    int cnt = 0;
    while (count < len)
    {
        int ret = recv(clnt_sock, buffer, len - count, 0);
        if ((ret < 0) && (errno != EINTR) && (errno != EWOULDBLOCK) && (errno != EAGAIN) && (errno != 0)) //出错了
        {
            cout << "connect failure! errno:"<<errno<< endl;
            return -2;
        }
        else if((ret < 0) && (errno == EINTR))  //被中断，继续
        {
            continue;
        }
        else if((ret < 0) && ((errno == EWOULDBLOCK) || (errno == EAGAIN) || (errno == 0)))  //接收超时，先返回（-1）
        {
            if(count)//接收到一半超时了，则出错了，接不上了！
            {
                if(cnt>20)
                {
                    return -2;
                }
                cnt++;
                continue;
            }
            return -1; //没接收到数据就超时了，正常，先返回
        }
        else if(ret == 0)  //socket被关闭
        {
            cout << "close socekt by the other side!"<< endl;
            return -2;
        }
        count += ret;
        buffer += ret;
    }
    return count;
}

void VSocket::recv_prior_map_file(int len)
{
	cout << "length = " << len << endl;
	const int num = len * sizeof(VPointI);
	char* buf = new char[num];
        int ret = recv_large(num, buf);
	if(ret == -1) //正常超时
		return;
	else if(ret == 0 || ret == -2) //异常
                return;
	  //解析
	vector<VPointI> cloud;
	cloud.clear();

        char *pp = buf;
        VPointI* pmsg;
	for (int i = 0; i < len; i++)
        {
		pmsg = (VPointI*)pp;
                pp += sizeof(VPointI);

                cloud.push_back(*pmsg);
	}
	ofstream infile("map.bt", std::ios::out | std::ios::binary);
	if(!infile.is_open())
		return;
	for(size_t i = 0; i < cloud.size(); ++i){
		/*cout << cloud[i].x << ","
		     << cloud[i].y << ","
		     << cloud[i].z << ","
		     << cloud[i].intensity << endl;*/
		infile.write((char*)&cloud[i], sizeof(VPointI));
	}
	infile.close();
	delete []buf;
	m_is_locate = false;//接收数据完毕
}


int VSocket::Accept(){	
	if(false == m_bissuccessInit)	
		return -1;
	
	//读取服务器传回的数据
	char buffer[1024];
	//int ret = recv(clnt_sock, buffer, sizeof(StartPackage),MSG_DONTWAIT);
	int ret = recv(clnt_sock, buffer, sizeof(CPackage), MSG_DONTWAIT);
	if(ret <= 0)
		return -1;
	CPackage* readbuf = (CPackage*)buffer;
	cout << "type = "<< readbuf->msgType <<endl;
	ret = 0;
	int readint = readbuf->msgLen;
	cout<<"readint"<<readbuf->msgLen<<endl;
	
	switch (readbuf->msgType)	{
		case CMD_START_ALGO:
		{
			memset(buffer, 0, 1024);
			recv(clnt_sock, buffer, (sizeof(char[32]) + 5 * sizeof(bool)), MSG_DONTWAIT);
			StartPackage* readbuf2 = (StartPackage*)buffer;
			isPointSaveServer = readbuf2->isPointSaveServer;
    			isPoseSaveServer = readbuf2->isPoseSaveServer;
   			isSaveNavMap = readbuf2->isSaveNavMap;
   			isPointTransBack = readbuf2->isPointTransBack;
            isPoseTransBack = readbuf2->isPoseTransBack;
			current_path = "/home/robot/robot_ws/data/";
			current_path += readbuf2->time;
			string time = readbuf2->time;
			priorfilename = readbuf2->time;
			if((!isPoseSaveServer)&&(!isPointSaveServer))
			{
				int status = mkdir(current_path.c_str(),0777);//chuang jian wen jian jia 
				//cout << "isPointSaveServer =" << isPointSaveServer << endl;
				//cout << "isPoseSaveServer =" << isPoseSaveServer << endl;
				//cout << "isSaveNavMap =" << isSaveNavMap << endl;
				//cout << "isPointTransBack =" << isPointTransBack << endl;
				//cout << "isPoseTransBack =" << isPoseTransBack << endl;
				VFileOperation::saveInfoTXT(current_path + "/.info.txt", time, readbuf2->isSaveNavMap);
			}
			ret = 6;
			break;
		}
		case CMD_STOP_END:
		{
			ret = 5;
			break; 
		}
		case CMD_STOP_ALGO:
		{
			ret = 7;
			break;
		}
		case CMD_START_LOCATION:
		{
			memset(buffer ,0, 1024);
			recv(clnt_sock, buffer, (sizeof(CmdType) + sizeof(unsigned int) + sizeof(char[32]) + 2 * sizeof(bool)), MSG_DONTWAIT);
			LocatePackage* readbuf2 = (LocatePackage*)buffer;
			priorfilename = readbuf2->ScanID;
			//cout<<ScanID<<endl;
			//cout<<readbuf2->isPointTransBack<<endl;
			//cout<<readbuf2->isPoseTransBack<<endl;
			//cout<<"定位开始"<<endl;
			ret = 8;
			m_is_locate = true;
			break;
		}
		case CMD_STOP_LOCATION:
		{
		        //stop to locate
		        if(false == m_is_locate)
		                ret = 9;
		         break;
		}
		default:
		{
			ret = 0;
			break;
		}
	}
	return ret;
}

bool VSocket::SendPos(char *pointpose,const int length)
{
	if(false == m_bissuccessInit)	
		return false;
	
	int ret = send(clnt_sock, pointpose, length, 0);
	if(-1 == ret)
		return false;
	return true;
}

bool VSocket::SendPC(char *pointcloud,const int length)
{
	//cout<<"send the point cloud data !"<<endl;
	if(false == m_bissuccessInit)	
		return false;

	int ret = send(clnt_sock, pointcloud, length, 0);
	if(-1 == ret)
		return false;
	return true;
}

void VSocket::close_current_sock()
{
	close(clnt_sock);
}

void VSocket::Terminate()
{
	if(false == m_bissuccessInit)	
		return;
	//关闭套接字
	close(clnt_sock);
	close(serv_sock);
}
/*
void VSocket::createDir(const char* path)
{
   int isCreate = mkdir(path,S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
   if( !isCreate )
   	printf("create path:%s\n",path);
   else
  	printf("create path failed! error code : %s \n",isCreate,path);
}*/

