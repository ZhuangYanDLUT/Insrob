#include <ros/ros.h>
#include <iostream>
#include <ros/console.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include "nav_msgs/Odometry.h"
#include <time.h>
#include <sstream>
#include "gps_pub/imu_gps.h"
#include "Base_control/Motorfeedback.h"


#include <geometry_msgs/PoseStamped.h>

#include <fstream>
#include <iomanip>



bool boolGPSWrite = false;
bool boolPoseWrite = false;
bool boolGPStoPoseWrite= false;

char filesys[256] = "/home/robot/";
char filesys1[256] = "/home/robot/";
char filesys2[256] = "/home/robot/";
std::ofstream GPSWrite,PoseWrite,GPStoPoseWrite;

std::string strNodeName = "show_pose";

ros::Subscriber* g_pvechileSubscriber;
ros::Subscriber* g_pvechileGPSSubscriber;

ros::Publisher* g_pPoseShowPublisher;
ros::Publisher* g_pPosevechileShowPublisher;
ros::Publisher* g_pPoseGPSShowPublisher;



void GPSxyToGlobalxy( double longtitude,double latitude,double &x, double &y )
{
		const double L0 = 121.5215999;
		const double C = 6399596.65198801;		// Œ«µã×ÓÎçÈŠÇúÂÊ°ëŸ¶
		const double ee = 0.00673951819500;

		const double PI = 3.1415926535;

		double sinB = sin(PI/180*latitude);
		double cosB = cos(PI/180*latitude);
		double X = 111134.0047*latitude-(32009.8575*sinB+133.9602*sinB*sinB*sinB
			+0.6976*sinB*sinB*sinB*sinB*sinB+0.0039*sinB*sinB*sinB*sinB*sinB*sinB*sinB)*cosB;
		double t = tan(PI/180*latitude);
		double l = longtitude - L0;
		double m = PI/180*l*cosB;
		double nn = ee*cosB*cosB;
		double N = C/sqrt(1+nn);
		x = X + N*t*(0.5*m*m)+(double)1/24*(5-t*t+4*nn*nn)*m*m*m*m
			+(double)1/720*(61-58*t*t+t*t*t*t*m*m*m*m*m*m);
		y = N*(m+(double)1/6*(1-t*t+nn)*m*m*m+(double)1/120*(5-18*t*t+t*t*t*t+14*nn-58*nn*t*t)*m*m*m*m*m);
}
/*
geometry_msgs::PoseArray PoseArrayVechile;
geometry_msgs::PoseArray PoseArrayGPS;
bool firsttime = true;
double now_x = 0;
double now_y = 0;
double last_x = 0;
double last_y = 0;
double dis = 0.0;

void PoseCallback(const transformation::VehiclePose& msg)
{
	//std::cout<<"PoseCallback PoseCallback  "<<dis<<std::endl;
	//static int poseCounter = 0;
	//poseCounter++;
	
	tf::Transform transform;
    tf::Quaternion q;
    q.setRPY(msg.roll,msg.pitch,msg.heading);
    transform.setRotation(q);
        

    //std::cout<<"msg.heading, msg.pitch, msg.roll "<<msg.heading<<" "<<msg.pitch<<" "<<msg.roll<<std::endl;



	
    geometry_msgs::PoseStamped poseShow;
    poseShow.header.stamp = msg.header.stamp;
    poseShow.header.frame_id = "/world";
    poseShow.pose.position.x = msg.x;
    poseShow.pose.position.y = msg.y;
    poseShow.pose.position.z = msg.z;
    poseShow.pose.orientation.x = transform.getRotation().getX();
    poseShow.pose.orientation.y = transform.getRotation().getY();
    poseShow.pose.orientation.z = transform.getRotation().getZ();
    poseShow.pose.orientation.w = transform.getRotation().getW();

     g_pPoseShowPublisher->publish(poseShow);
	
	
	double X,Y;
    GPSxyToGlobalxy(msg.longitude,msg.latitude,X,Y);
    Y = -Y;

    geometry_msgs::Pose tempose;
    tempose.position.x = msg.x;
    tempose.position.y = msg.y;
    tempose.position.z = msg.z;
    tempose.orientation.x = transform.getRotation().getX();
    tempose.orientation.y = transform.getRotation().getY();
    tempose.orientation.z = transform.getRotation().getZ();
    tempose.orientation.w = transform.getRotation().getW();

	
	geometry_msgs::Pose tempose1;
    tempose1.position.x = X;
    tempose1.position.y = -Y;
    tempose1.position.z = msg.z;
    tempose1.orientation.x = transform.getRotation().getX();
    tempose1.orientation.y = transform.getRotation().getY();
    tempose1.orientation.z = transform.getRotation().getZ();
    tempose1.orientation.w = transform.getRotation().getW();

	
	double tempX = X - now_x;
    double tempY = Y - now_y;
  	dis = (tempX - last_x) * (tempX - last_x) + (tempY - last_y) * (tempY - last_y);
	//std::cout<<"dis "<<tempX<<" "<<tempY<<" "<<last_x<<" "<<last_y<<" "<<dis<<std::endl;
	
	
	if(dis>1.0 || firsttime)
    {
       if(firsttime)
       {
	 	 now_x = X;
         now_y = Y;

         tempose1.position.x = 0;
    	 tempose1.position.y = 0;
		

		if(boolGPSWrite)
		{		   
		 GPSWrite<<setiosflags(std::ios::fixed)<<std::setprecision(10)<<msg.longitude<<" "<<setiosflags(std::ios::fixed)<<std::setprecision(10)<<msg.latitude<<" "<<setiosflags(std::ios::fixed)<<std::setprecision(10)<<msg.altitude<<" ";
		}
		
		if(boolPoseWrite)
		{
		 	PoseWrite<<tempose.position.x<<" "<<tempose.position.y<<" ";
		}

		if(boolGPStoPoseWrite)
		{
         		GPStoPoseWrite<<tempose1.position.x<<" "<<tempose1.position.y<<" ";
		}
    	 
         firsttime =false;
       }
       else
       {
       
	     tempose1.position.x = X - now_x;
    	 tempose1.position.y = Y - now_y;
		 last_x = tempose1.position.x;
         last_y = tempose1.position.y;
		   
		if(boolGPSWrite)
		{
			GPSWrite<<setiosflags(std::ios::fixed)<<std::setprecision(10)<<msg.longitude<<" "<<setiosflags(std::ios::fixed)<<std::setprecision(10)<<msg.latitude<<" "<<setiosflags(std::ios::fixed)<<std::setprecision(10)<<msg.altitude<<" ";
		}

		if(boolPoseWrite)
		{
			PoseWrite<<tempose.position.x<<" "<<tempose.position.y<<" ";
		}
	
		if(boolGPStoPoseWrite)
		{
          		GPStoPoseWrite<<tempose1.position.x<<" "<<tempose1.position.y<<" "; 
		}
		   
       }


	    PoseArrayVechile.poses.push_back(tempose);
        PoseArrayGPS.poses.push_back(tempose1);

        g_pPosevechileShowPublisher->publish(PoseArrayVechile);
        g_pPoseGPSShowPublisher->publish(PoseArrayGPS);
    }

}
*/

bool poseok=false;
bool gpsok=false;

nav_msgs::Odometry pubmsg;
void PoseCallback(const nav_msgs::Odometry& msg)
{
	pubmsg = msg;
	poseok = true;

    double roll=0.0,pitch=0.0,yaw=0.0;
    tf::Matrix3x3(tf::Quaternion(pubmsg.pose.pose.orientation.x, pubmsg.pose.pose.orientation.y, pubmsg.pose.pose.orientation.z, pubmsg.pose.pose.orientation.w)).getRPY(roll, pitch, yaw);

    if(poseok && boolPoseWrite)
    {
        PoseWrite<<pubmsg.pose.pose.position.x<<" "<<pubmsg.pose.pose.position.y<<" "<<0.0<<" "
     <<yaw<<" "<<pitch<<" "<<roll<<" "<<0<<" "<<0<<" "<<1<<" "<<1<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<std::endl;
    }

	g_pPoseShowPublisher->publish(pubmsg);
}


geometry_msgs::PoseArray PoseArrayGPS;
bool firsttime = true;
double now_x = 0;
double now_y = 0;
double last_x = 0;
double last_y = 0;
double dis = 0.0;

void GPSCallback(const gps_pub::imu_gps& msg)
{
    std::stringstream is;
    is<<msg.longitude;
    double longitude;
    is>>longitude;
	longitude = longitude/100.0;

    std::stringstream is1;
    is1<<msg.latitude;
    double latitude;
    is1>>latitude;
	latitude = latitude/100.0;

	std::stringstream is2;
    is2<<msg.altitude;
    double altitude;
    is2>>altitude;
	altitude = altitude/1.0;


    double roll=0.0,pitch=0.0,yaw=0.0;
    tf::Matrix3x3(tf::Quaternion(pubmsg.pose.pose.orientation.x, pubmsg.pose.pose.orientation.y, pubmsg.pose.pose.orientation.z, pubmsg.pose.pose.orientation.w)).getRPY(roll, pitch, yaw);


    std::stringstream status;
    status<<msg.GPSstatus;
    int GPSstatus;
    status>>GPSstatus;

	std::stringstream gpsnum;
    gpsnum<<msg.GPSstatus;
    int Gpsnum;
    gpsnum>>Gpsnum;

    if(boolGPSWrite && poseok)
	{	
         if(GPSstatus==4 || GPSstatus==5)
         {
            
             GPSWrite<<pubmsg.pose.pose.position.x<<" "<<pubmsg.pose.pose.position.y<<" "<<0.0<<" "
         <<yaw<<" "<<pitch<<" "<<roll<<" "
         <<GPSstatus<<" "<<Gpsnum<<" "<<1<<" "<<1<<" "<<" "<<setiosflags(std::ios::fixed)<<std::setprecision(10)<<latitude<<" "
         <<setiosflags(std::ios::fixed)<<std::setprecision(10)<<longitude<<" "<<setiosflags(std::ios::fixed)<<std::setprecision(10)<<altitude<<" "
         <<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<std::endl;	

         }	
         else
         {
             GPSWrite<<pubmsg.pose.pose.position.x<<" "<<pubmsg.pose.pose.position.y<<" "<<0.0<<" "
 	 <<yaw<<" "<<pitch<<" "<<roll<<" "<<GPSstatus<<" "<<Gpsnum<<" "<<1<<" "<<1<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<std::endl;

         }   

	}



    double X,Y;
    GPSxyToGlobalxy(longitude,latitude,X,Y);
    Y = -Y;
	
    geometry_msgs::Pose tempose1;
    tempose1.position.x = X;
    tempose1.position.y = -Y;
    tempose1.position.z = 0;
    tempose1.orientation.x = 0;
    tempose1.orientation.y = 0;
    tempose1.orientation.z = 0;
    tempose1.orientation.w = 1;

	
	double tempX = X - now_x;
    double tempY = Y - now_y;
  	dis = (tempX - last_x) * (tempX - last_x) + (tempY - last_y) * (tempY - last_y);
	//std::cout<<"dis "<<tempX<<" "<<tempY<<" "<<last_x<<" "<<last_y<<" "<<dis<<std::endl;
	//std::cout<<"dis "<<std::endl;
	
	if(dis>1.0 || firsttime)
    {
       if(firsttime)
       {
	 	 now_x = X;
         now_y = Y;

         tempose1.position.x = 0;
    	 tempose1.position.y = 0;
         firsttime =false;
	     //std::cout<<"first time................................................. "<<std::endl;
       }
       else
       {
       
	     tempose1.position.x = X - now_x;
    	 tempose1.position.y = Y - now_y;
		 last_x = tempose1.position.x;
         last_y = tempose1.position.y;
		 //std::cout<<"tempose1............................................................... "<<tempose1.position.x<<" "<<tempose1.position.y<<std::endl;
		   
       }
	}
	

	PoseArrayGPS.poses.push_back(tempose1);
	g_pPoseGPSShowPublisher->publish(PoseArrayGPS);

}


ros::Publisher PosePublisher;
int lastleftcount = 0;
int latsrightcount = 0;
float Robx = 0.0;
float Roby = 0.0;
float Robz = 0.0;
float Robyaw = 0.0;
float Sumyaw = 0.0;
bool isfirst = true;
float d = 0.2785;
float robwidth = 0.519;

void MotorCallback(const Base_control::Motorfeedback &msg)
{
    int leftback = msg.leftback * (-1);
    int leftfront = msg.leftfront * (-1);
    int rightback = msg.rightback;
    int rightfront = msg.rightfront;


    float averageleft = (leftback + leftfront)/2;
    float averageright = (rightback + rightfront)/2;

    if(isfirst)
    {
        lastleftcount = averageleft;
        latsrightcount = averageright;
        isfirst = false;
    }
    else
    {
        std::cout<<"averageleft "<<averageleft<<" averageright "<<averageright<<" lastleftcount "<<lastleftcount<<" latsrightcount "<<latsrightcount<<std::endl;
                
        float detaleftcount = float(averageleft - lastleftcount);
        float detarightcount = float(averageright - latsrightcount);

        std::cout<<"detaleftcount "<<detaleftcount<<" detarightcount "<<detarightcount<<std::endl;

        float disdetaleft = detaleftcount/(40.0 * 10000.0) * 3.1415926 * d;
        float disdetaright = detarightcount/(40.0 * 10000.0) * 3.1415926 * d;
                    
        std::cout<<" disdetaleft "<<disdetaleft<<" disdetaright "<<disdetaright<<std::endl;

        float detasunS = (disdetaleft + disdetaright)/2;
        float detath = (disdetaright - disdetaleft)/robwidth;

        std::cout<<"detasunS "<<detasunS<<" detath "<<detath<<std::endl;

        float detax = detasunS * cos(Sumyaw + detath * 0.5);
        float detay = detasunS * sin(Sumyaw + detath * 0.5);


        Robx = Robx + (detax * cos(Sumyaw) - detay *sin(Sumyaw));
        Roby = Roby + (detax * sin(Sumyaw) + detay *cos(Sumyaw));
        Robz = 0.0;

        Sumyaw = Sumyaw + detath;

        std::cout<<"before Robyaw "<<Sumyaw<<std::endl;

        float yawDegree = Sumyaw * 180 / 3.1415926;

        std::cout<<" yawDegree "<<yawDegree<<std::endl;

        if(yawDegree>180)
        {
            int shang = yawDegree/360.0;
            float yu = (yawDegree - 360.0 * shang);

            std::cout<<" shang "<<shang<<" yu "<<yu<<std::endl;

            if(yu>180)
            {
                Robyaw = (yu - 360)* 3.1415926 / 180.0;

            }
            else
            {

                Robyaw = yu * 3.1415926 / 180.0;
            }

        }
        else
        {
            if(yawDegree<-180)
            {
                yawDegree = yawDegree * (-1);
                int shang = yawDegree/360.0;
                float yu = (yawDegree - 360.0 * shang);

                if(yu>180)
                {
                    Robyaw = (yu - 360)* 3.1415926 / 180.0 * -1;
                }
                else
                {
                    Robyaw = yu * 3.1415926 / 180.0 * -1;

                }            
            }
            else
            {
                Robyaw = Sumyaw;                    
            }

        }

        
    lastleftcount = averageleft;
    latsrightcount = averageright;
    }


    float roll = 0.0;
    float pitch = 0.0;

    nav_msgs::Odometry Robpose;
    Robpose.header.stamp = ros::Time::now();
    Robpose.header.frame_id = "/camera_init";

    Robpose.pose.pose.position.x = Robx;
    Robpose.pose.pose.position.y = Roby;
    Robpose.pose.pose.position.z = Robz;

    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,Robyaw);

    Robpose.pose.pose.orientation.x = q.x;
    Robpose.pose.pose.orientation.y = q.y;
    Robpose.pose.pose.orientation.z = q.z;
    Robpose.pose.pose.orientation.w = q.w;

    PosePublisher.publish(Robpose);

}


int main(int argc, char** argv)
{
   

    char str_FID[128] = {0};
    char str_FID1[128] = {0};
	char str_FID2[128] = {0};
	
    time_t t;  //
    tm* local; //
    t = time(NULL); //��ȡĿǰ��ʱ��
    local = localtime(&t); //תΪ����ʱ��
    strftime(str_FID, 64, "%Y_%m_%d_%H_%M_%S", local);
    strcpy(str_FID1,str_FID);
	strcpy(str_FID2,str_FID);
	
    char filefinalname[64] = "_GPSWrite.txt";
    char filefinalname1[64] = "_PoseWrite.txt";
	char filefinalname2[64] = "_GPStoPoseWrite.txt";
	
    strcat(str_FID, filefinalname);
    strcat(str_FID1, filefinalname1);
	strcat(str_FID2, filefinalname2);
	
    strcat(filesys,str_FID);
    strcat(filesys1,str_FID1);
	strcat(filesys2,str_FID2);
	
    GPSWrite.open(filesys,std::ios_base::app);
    PoseWrite.open(filesys1,std::ios_base::app);
    GPStoPoseWrite.open(filesys2,std::ios_base::app);

	
    ros::init(argc, argv, strNodeName);
    ros::NodeHandle node;
	
	g_pvechileSubscriber = new ros::Subscriber;
	*g_pvechileSubscriber = node.subscribe("/air_pose", 100, &PoseCallback);


    g_pvechileGPSSubscriber= new ros::Subscriber;
	*g_pvechileGPSSubscriber = node.subscribe("gps_imu", 100, &GPSCallback);

	
 

    g_pPoseShowPublisher = new ros::Publisher;
    *g_pPoseShowPublisher = node.advertise<nav_msgs::Odometry>("PoseShow1", 20);
	
    //g_pPosevechileShowPublisher = new ros::Publisher;
    g_pPoseGPSShowPublisher = new ros::Publisher;
	
    //*g_pPosevechileShowPublisher = node.advertise<geometry_msgs::PoseArray>("PoseVechlieShow1", 20);
    *g_pPoseGPSShowPublisher = node.advertise<geometry_msgs::PoseArray>("PoseGPSShow1", 20);
     

    ros::Subscriber Motorsub = node.subscribe("Motorfeed",100, &MotorCallback);
    PosePublisher = node.advertise<nav_msgs::Odometry>("Motorpose", 20);

     /*PoseArrayVechile.header.stamp = ros::Time::now();
     PoseArrayVechile.header.frame_id = "/world";*/

     PoseArrayGPS.header.stamp = ros::Time::now();
     PoseArrayGPS.header.frame_id = "/camera_init";

   
     ros::spin();
	
     GPSWrite.close();
     PoseWrite.close();
	 GPStoPoseWrite.close();
}
