#include <Base_control/UART.h>


#include <bits/stdc++.h>
#include <unistd.h>//Unix标准函数定义
#include <termios.h>//PPSIX终端控制定义
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>//文件控制定义
#include <sys/io.h>
#include <stdarg.h>
#include <stdio.h>

using namespace std;

//×××××××××××××××串口初始化开始×××××××××××××××××
//---------------移植自CarVctrl-----------------
//×××××××××××××××××××××××××××××××××××××××××××××

const char* VportName = "/dev/ttyUSB1";//串口号

UART::UART()
{
	//weizichushihua
	lastleftcount = 0.0;
	latsrightcount = 0.0;
	Robx=Roby=Robz=Sumyaw=Robyaw=Robpitch=Robroll=Roblinear=Robtwist=0.0;
	isfirsttime = true;
	d = 0.2785;
	robwidth = 0.519;



	fd = -1;
	baud = 115200;//波特率
	nBits = 8;
	nEvent = 'N';
	nStop = 1;
	
	Maxtrytimes = 20;
	AllinitsetOK = false;

	if(Init(VportName))
	{
		printf("serial OK\n");
		AllinitsetOK = true;
	}
	else
	{
		printf("serial failed\n");
		AllinitsetOK = false;
	}
}

UART::~UART()
{
	if(fd!=-1)
	{
		close(fd);
	}
}

bool UART::Init(const char* VportName)
{
        printf("VportName:%s",VportName);
	fd = open(VportName, O_RDWR|O_NOCTTY|O_NDELAY);//非阻塞模式
	int trytimes = 0;
	while(fd<0 && trytimes<Maxtrytimes)
	{

		std::cout<<"serial not open,open again once more___"<<std::endl;
		usleep(200000);//200ms 重复执行并判断10次看看能不能连接上
		fd = open(VportName, O_RDWR|O_NOCTTY|O_NDELAY);
		trytimes++;
	}

	if(fd == -1)
	{
		perror("Can't Open Serial Port\n");
         	return false;
	}

	
	 printf("%s is open!\n",VportName);

	//判定串口是否占用终端
   	 if(isatty(STDIN_FILENO)==0)
       		 printf("standard input is not a terminal device\n");
  	  else
        	printf("isatty success!\n");
	
	 if(set_opt())
	 {
		printf("open and set success\n");
		return true;
	 }
	 else
	 {
		printf("set failed\n");
		return false;
	 }
}

bool UART::set_opt()
{
	struct termios newtio,oldtio;
	if ( tcgetattr( fd,&oldtio) != 0) 
	{ 
        perror("SetupSerial 1");
        return false;
    }
	bzero( &newtio, sizeof(newtio));
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE; 
	
	switch(nBits)
    {
    	case 7:
        	newtio.c_cflag |= CS7;
        break;
    	case 8:
        	newtio.c_cflag |= CS8;
        break;
    }

	switch( nEvent )
    {
    	case 'O':                     //奇校验
        	newtio.c_cflag |= PARENB;
        	newtio.c_cflag |= PARODD;
        	newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    	case 'E':                     //偶校验
        	newtio.c_iflag |= (INPCK | ISTRIP);
        	newtio.c_cflag |= PARENB;
        	newtio.c_cflag &= ~PARODD;
        break;
    	case 'N':                    //无校验
        	newtio.c_cflag &= ~PARENB;
        break;
    }
	
	switch(baud)
    {
    	case 2400:
        	cfsetispeed(&newtio, B2400);
        	cfsetospeed(&newtio, B2400);
        break;
    	case 4800:
        	cfsetispeed(&newtio, B4800);
        	cfsetospeed(&newtio, B4800);
        break;
    	case 9600:
        	cfsetispeed(&newtio, B9600);
        	cfsetospeed(&newtio, B9600);
        break;
    	case 115200:
        	cfsetispeed(&newtio, B115200);
        	cfsetospeed(&newtio, B115200);
        break;
    	default:
        	cfsetispeed(&newtio, B9600);
        	cfsetospeed(&newtio, B9600);
        break;
    }
	
	if(nStop==1)
	{
		newtio.c_cflag &=  ~CSTOPB;
	}
	else
	{
		if(nStop==2)
		{
			newtio.c_cflag |=  CSTOPB;
		}
	}
	
	newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd,TCIFLUSH);
	
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return false;
    }
	return true;
}
//×××××××××××××××××××××××××××××××××××××××××××××
//---------------串口初始化结束-----------------
//×××××××××××××××××××××××××××××××××××××××××××××
bool UART::Setspeed()//发送数据给串口
{
	if(AllinitsetOK)
	{
		usleep(20000);//20ms
		if(write(fd,u,31)>0)//每次写数据时必须要等待串口接受时间，大概是10~50ms
		{
			//NSpeed_stall = 0;
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		std::cout<<"serial set not ok  "<<std::endl;
	}
}


const char PoseCmd[] ={0xAA,0x01,0x01,0x1F,0x00,0x03,0x3E,0x0D};

void UART::Getpose(float &x,float &y,float &z,float &yaw,
	  	   				float &pitch,float &roll,float &linear,
	  	   				float &twist,int &backleftcount,int &frontleftcount,
	  	   				int &backrightcount,int &frontrightcount)
{
	//std::cout<<"get in...... "<<std::endl;
	if(AllinitsetOK)
    {
    	std::cout<<"ggggggggg................ "<<std::endl;

        tcflush(fd,TCIFLUSH);
       
  //       usleep(20000);
  //       char Readdata[32] = {0};
		// int readcount = read(fd,Readdata,32);
		// if(readcount>0)
		// {
		// 	std::cout<<"zangshuju "<<std::endl;
		// }


        usleep(20000);//20ms
		if(write(fd,PoseCmd,sizeof(PoseCmd))>0)//每次写数据时必须要等待串口接受时间，大概是10~50ms
		{
			std::cout<<"ggggggggg1222222222222222222................ "<<std::endl;
			
			usleep(30000);
			//NSpeed_stall = 0;
			unsigned char Readdata[32] = {0};
			int readcount = read(fd,Readdata,32);
			usleep(30000);

			if(readcount>0)
			{
				for(int i=0;i<readcount;i++)
				{
					printf("0x%x ",Readdata[i]);

				}

				printf("\n");

				if(readcount!=24)
				{
					std::cout<<"wrong shuju&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& "<<std::endl;
					return;
				}

				int leftback = (Readdata[5]) | (Readdata[6]<<8) 
								| (Readdata[7]<<16) | (Readdata[8]<<24);

				int rightback = (Readdata[9]) | (Readdata[10]<<8) 
								| (Readdata[11]<<16) | (Readdata[12]<<24);
				
				int leftfront = (Readdata[13]) | (Readdata[14]<<8) 
								| (Readdata[15]<<16) | (Readdata[16]<<24);

				int rightfront = (Readdata[17]) | (Readdata[18]<<8) 
								| (Readdata[19]<<16) | (Readdata[20]<<24);


				std::cout<<"houlun left________________________________ "<<std::endl;

				std::cout<<leftback<<std::endl;
				backleftcount = leftback;

				std::cout<<"qianlun left_111111111------------------- "<<std::endl;

				std::cout<<leftfront<<std::endl;
				frontleftcount = leftfront;

				std::cout<<"houlun right__2222222-------------------- "<<std::endl;

				std::cout<<rightback<<std::endl;
				backrightcount = rightback;

				std::cout<<"qianlun right__333333----------------------- "<<std::endl;

				std::cout<<rightfront<<std::endl;
				frontrightcount = rightfront;

				leftback = leftback * (-1);
				leftfront = leftfront * (-1);


				std::cout<<" leftback "<<leftback<<" leftfront "<<leftfront<<" rightback "<<rightback
						 <<" rightfront "<<rightfront<<std::endl;

				float averageleft = (leftback + leftfront)/2;
				float averageright = (rightback + rightfront)/2;

				if(isfirsttime)
				{
					lastleftcount = averageleft;
					latsrightcount = averageright;
					isfirsttime = false;
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

						// Robyaw =  (yu + 180 ) * 3.1415926 / 180.0;

						// std::cout<<" Robyaw "<<Robyaw<<std::endl;

						// Robyaw = -1 * Robyaw;
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



							// yawDegree = yawDegree * (-1);
							// int shang = yawDegree/180.0;
							// float yu = 180.0 - (yawDegree - 180.0 * shang);

							// std::cout<<" shang "<<shang<<" yu "<<yu<<std::endl;

							// Robyaw = yu * 3.1415926 / 180.0;

							// std::cout<<" Robyaw "<<Robyaw<<std::endl;

							// Robyaw = -1 * Robyaw;
						}
						else
						{
							Robyaw = Sumyaw;					
						}

					}

					Robpitch = 0.0;
					Robroll = 0.0;
					Roblinear = 0.0;
					Robtwist = 0.0;

					x = Robx;
					y = Roby;
				    z = Robz;
				    yaw = Robyaw;
				    pitch = Robpitch;
				    roll = Robroll;
				    linear = Roblinear;
				    twist = Robtwist;

				    lastleftcount = averageleft;
					latsrightcount = averageright;
				}

			}
			else
			{
				std::cout<<"serial write not success "<<std::endl;
			}
		}
		else
		{
			std::cout<<"serial write is not Ok "<<std::endl;
		}
    }
    else
    {
    	std::cout<<"serial setting is not Ok "<<std::endl;
    }

}

