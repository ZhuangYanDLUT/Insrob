#include <ros/ros.h>
#include "Base_control/UART.h"
#include <iostream>
#include <bits/stdc++.h>
#include <signal.h>
#include <time.h>
#include <boost/thread/mutex.hpp>
#include "Base_control/Insrobpose.h"
//--服务头文件（之前创建的服务文件中生成的：包/服务节点）
#include "Base_control/ControlCmd.h"
#include "Base_control/Motorfeedback.h"

#include <unistd.h>//Unix标准函数定义
#include <termios.h>//PPSIX终端控制定义
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>//文件控制定义
#include <sys/io.h>
#include <stdarg.h>
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

using namespace std;

ros::Publisher Robposepublisher;
ros::Publisher Motorfeedbackpuber;

UART UART_CTRL;

const int clockTime = 200;//定时器100ms
boost::mutex m_mtx;

//××××××××××××××××××测试函数×××××××××××××××××××××××××
const unsigned short crc_ta[256] = { 
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

bool CRC16(unsigned char *ptr, unsigned long len,unsigned char *crc1,unsigned char *crc2)//CRC函数输入28位2个字节，计算crc得到能用的两个连续char
{
        unsigned short crc;
	unsigned char da;
	crc = 0;
	while(len-- != 0)
	{
	  da = (unsigned char)(crc >> 8);
	  crc <<= 8;
	  crc ^= crc_ta[da^*ptr];
	  ptr++;
	}
	*crc1=(unsigned char)crc;
        *crc2=(unsigned char)(crc>>8);
}

bool transaction(int vel,int omiga,unsigned char *vel1,unsigned char *vel2,unsigned char *omiga1,unsigned char *omiga2)//速度转换16进制
{
 if(omiga>0)  omiga=(omiga/10)+50;
 else if(omiga==0) omiga=0;
 else omiga=(omiga/10)-50; 
*vel1=(unsigned char)vel;
 *vel2=(unsigned char)(vel>>8);
 *omiga1=(unsigned char)omiga;
 *omiga2=(unsigned char)(omiga>>8); 
}
//形成送给CRC进行校验的11个字符数组
bool joint1(unsigned char *vel1,unsigned char *vel2,unsigned char *omiga1,unsigned char *omiga2,unsigned char *ptr)
{
    ptr[0]=0xAA;ptr[1]=0x0F;ptr[2]=0x01;ptr[3]=0x2F;ptr[4]=0x17;ptr[5]=0x00;ptr[6]=0x00;
    ptr[7]=*vel1;ptr[8]=*vel2;ptr[9]=*omiga1;ptr[10]=*omiga2;ptr[11]=0xF1;ptr[12]=0xFE;
    ptr[13]=0x97;ptr[14]=0x00;ptr[15]=0xFB;ptr[16]=0xFF;ptr[17]=0x0C;ptr[18]=0x00;ptr[19]=0xFB;ptr[20]=0x01;
    ptr[21]=0xE3;ptr[22]=0x01;ptr[23]=0x61;ptr[24]=0x00;ptr[25]=0x00;ptr[26]=0x1F;ptr[27]=0x00;
}
//拼接成最终指令
bool joint2(unsigned char *ptr,unsigned char *crc1,unsigned char *crc2,unsigned char *u)
{
    int i;
    for(i=0;i<=27;i++) u[i]=ptr[i];
    u[28]=*crc1;u[29]=*crc2;u[30]=0x0D;
}
bool function_transction(unsigned char *u,int vel,int omiga)//in:u[14] vel omiga   out:u[14]
{
   unsigned char ptr[28]; 
   unsigned char vel1,vel2,omiga1,omiga2,crc1,crc2;
   transaction(vel,omiga,&vel1,&vel2,&omiga1,&omiga2);
   joint1(&vel1,&vel2,&omiga1,&omiga2,ptr);
   CRC16(ptr,28,&crc1,&crc2);
   joint2(ptr,&crc1,&crc2,u);
}
//××××××××××××××××测试函数结束×××××××××××××××××××××××


//-----------------回调函数---------------------------
//*********包名::服务名::请求/服务 类定义*************
//---------------------------------------------------
bool handlefunction(Base_control::ControlCmd::Request  &req,Base_control::ControlCmd::Response &res)
{
     
     //res.zz=req.xx+req.yy;
	
	//std::cout<<"recv................................................"<<std::endl;
	//std::cout<<"xiansudu "<<req.xx<<"jiaosudu  "<<req.yy<<std::endl;



     unsigned char u[31];
     function_transction(u,req.xx,req.yy);
	
	 
     
     if(UART_CTRL.AllinitsetOK)
  	 {
  		  std::cout<<"test  ++++ >>>>"<<std::endl;
		    UART_CTRL.u=u;

		    // m_mtx.lock();
    	  if(UART_CTRL.Setspeed())
    	  {
				  // m_mtx.unlock();
    			return true;
    	  }
    	  else
    	  {
				  // m_mtx.unlock();
    			return false;
    	  }
  	}
  	else
  	{
  		  std::cout<<"UART_CTRL set not OK "<<std::endl;
          return false;
  	}

     //for(int i=0;i<=13;i++) printf("%x    ",u[i]);//调试看结果
     //ROS_INFO("result: %ld",(long int)res.zz);
     //return true;//true
}



void timerThreadSelfDesigned(union sigval v)
{
  /*  std::cout<<"get into timer ............................. "<<std::endl;
    float x,y,z,yaw,pitch,roll,linear,twist;

    //m_mtx.lock();
	//boost::mutex::scoped_lock(m_mtx);

    int leftback,leftfront,rightback,rightfront;

    Base_control::Motorfeedback msg;
    UART_CTRL.Getpose(x,y,z,yaw,pitch,roll,linear,twist,leftback,leftfront,rightback,rightfront);
    msg.leftback = leftback;
    msg.leftfront = leftfront;
    msg.rightback = rightback;
    msg.rightfront = rightfront;

    nav_msgs::Odometry Robpose;
    Robpose.header.stamp = ros::Time::now();
    Robpose.header.frame_id = "/camera_init";

    Robpose.pose.pose.position.x = x;
    Robpose.pose.pose.position.y = y;
    Robpose.pose.pose.position.z = z;

    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);

    Robpose.pose.pose.orientation.x = q.x;
    Robpose.pose.pose.orientation.y = q.y;
    Robpose.pose.pose.orientation.z = q.z;
    Robpose.pose.pose.orientation.w = q.w;

    Robposepublisher.publish(Robpose);
    Motorfeedbackpuber.publish(msg);

    std::cout<<"---------------------------- pose pose pose --------------------"<<std::endl;
    std::cout<<x<<" "<<y<<" "<<yaw<<" "<<linear<<" "<<twist<<std::endl;
    //m_mtx.unlock();*/

}




//-----------------服务主题函数--------------------------- 
int main(int argc, char **argv)
{
   ros::init(argc, argv, "app_Controlservice");
   ros::NodeHandle n;//句柄语句

   
   ros::ServiceServer service = n.advertiseService("Controler", handlefunction);
   Robposepublisher = n.advertise<nav_msgs::Odometry>("Insrobpose", 20);
   Motorfeedbackpuber = n.advertise<Base_control::Motorfeedback>("Motorfeed", 20);

   timer_t timerID;
   struct sigevent evp;
   memset(&evp, 0, sizeof(struct sigevent));
   evp.sigev_value.sival_int = 111;
   evp.sigev_notify = SIGEV_THREAD;


   evp.sigev_notify_function = timerThreadSelfDesigned;

   if(timer_create(CLOCK_REALTIME, &evp, &timerID) == -1)
   {
      perror("fail to timer_create");
      exit(-1);
   }

   struct itimerspec it;
   it.it_interval.tv_sec = 0.0;
   it.it_interval.tv_nsec = clockTime*1000000;
   it.it_value.tv_sec = 0;
   it.it_value.tv_nsec = 200 * 1000000;//clockTime*1000000;

   if (timer_settime(timerID, 0, &it, NULL) == -1)
   {
      perror("fail to timer_settime");
      exit(-1);
   }

   ROS_INFO("service start!!!");
   
   ros::MultiThreadedSpinner spinner(2);
   while (ros::ok()) {
    spinner.spin();
   }
	
   timer_delete(timerID);
   
   return 0;
}

