#ifndef UART_H
#define UART_H



class UART
{
    public:
           UART();
           ~UART();
           bool Init(const char* portname);
		   bool set_opt();
           bool Setspeed();
	  	   void Getpose(float &x,float &y,float &z,float &yaw,
	  	   				float &pitch,float &roll,float &linear,
	  	   				float &twist,int &backleftcount,int &frontleftcount,
	  	   				int &backrightcount,int &frontrightcount);
		   
		   
           int fd;//以读写方式打开串口
		   int baud;//波特率
		   int nBits;//数据位
		   int nStop;//停止位
		   char nEvent;//校验位
		   bool AllinitsetOK;
		   int Maxtrytimes;

		   int lastleftcount;
		   int latsrightcount;
		   float Robx;
		   float Roby;
		   float Robz;
		   float Robyaw;
		   float Sumyaw;
		   float Robpitch;
		   float Robroll;
		   float Roblinear;
		   float Robtwist;
		   bool isfirsttime;
		   float d;
		   float robwidth;


		   unsigned char *u;

	   //int NSpeed_stall;
};
#endif 
