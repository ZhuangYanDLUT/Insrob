#include <iostream>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h> /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */
using namespace std;

#define FALSE 0
#define TRUE  1

/*****************************************************************
* 名称：           serialPortOpen
* 功能：           打开串口并返回串口设备文件描述
* 入口参数：       fd : 文件描述符
*                  port :串口号(ttyS0,ttyS1,ttyS2)
* 出口参数：       正确返回1，错误返回0
*****************************************************************/
int serialPortOpen(int &fd,const char* port){
		fd = open(port, O_RDONLY | O_NOCTTY | O_NDELAY);
		if(fd == -1){
				perror("unable to open serial port");
				close(fd);
				return(FALSE);
		}
		//判断串口的状态是否为阻塞状态
		if(fcntl(fd, F_SETFL, 0) < 0){
				 cout << "fcntl failed!" << endl;
				 return(FALSE);
		}else{
				 cout << "fcntl=" << fcntl(fd, F_SETFL,0) << endl;
		}
		//测试是否为终端设备
		if(0 == isatty(STDIN_FILENO)){
			 cout << "standard input is not a terminal device" << endl;
					 return(FALSE);
			 }else{
					 cout << "isatty success!" << endl;
		}
		cout << "fd->open=" << fd << endl;
		return(TRUE);
}
/*******************************************************************
* 名称：          serialPortSet
* 功能：          设置串口数据位，停止位和效验位
* 入口参数：      fd         串口文件描述符
*                 speed      串口波特率
*                 flow_ctrl  数据流控制
*                 databits   数据位   取值为 7 或者8
*                 stopbits   停止位   取值为 1 或者2
*                 parity     校验类型 取值为N,E,O,S
*出口参数：       正确返回为1，错误返回为0
*******************************************************************/
int serialPortSet(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity){
		unsigned int i;
		int   speed_arr[] = {B1000000,B921600,B576000,B500000,B460800,B230400,B115200,
												 B57600,B38400,B19200,B9600,B4800,B2400,B1200,B300};
		int   name_arr[] =  {1000000,921600,576000,500000,460800,230400,115200,
												 57600,38400,19200,9600,4800,2400,1200,300};
		struct termios options;
		/* tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数,还可以测试配置是否正确，
		 * 该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
		*/
		if  ( tcgetattr( fd,&options)  !=  0)
			 {
					perror("Setup Serial 1");
					return(FALSE);
			 }
		//设置串口输入波特率和输出波特率
		bool baudFlag=false;
		for( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++){
				if  (speed == name_arr[i]){
						baudFlag = true;
						cfsetispeed(&options, speed_arr[i]);
						cfsetospeed(&options, speed_arr[i]);
				}
		}
		if(baudFlag==false){
				cout << "No such baudrate!" << endl;
				return (FALSE);
		}
		//修改控制模式，保证程序不会占用串口
		options.c_cflag |= CLOCAL;
		//修改控制模式，使得能够从串口中读取输入数据
		options.c_cflag |= CREAD;
		//设置数据流控制
		switch(flow_ctrl)
		{
			 case 0 ://不使用流控制
					options.c_cflag &= ~CRTSCTS;
					break;
			 case 1 ://使用硬件流控制
					options.c_cflag |= CRTSCTS;
					break;
			 case 2 ://使用软件流控制
					options.c_cflag |= IXON | IXOFF | IXANY;
					break;
			 default:
					fprintf(stderr,"Unsupported flow control\n");
					return (FALSE);
		}
		//设置数据位
		options.c_cflag &= ~CSIZE; //屏蔽其他标志位
		switch (databits)
		{
			 case 5:
					options.c_cflag |= CS5;
					break;
			 case 6:
					options.c_cflag |= CS6;
					break;
			 case 7:
					options.c_cflag |= CS7;
					break;
			 case 8:
					options.c_cflag |= CS8;
					break;
			 default:
					fprintf(stderr,"Unsupported data size\n");
					return (FALSE);
		}
		//设置校验位
		switch (parity)
		{
			 case 'n':
			 case 'N': //无奇偶校验位。
					options.c_cflag &= ~PARENB;
					options.c_iflag &= ~INPCK;
					break;
			 case 'o':
			 case 'O'://设置为奇校验
					options.c_cflag |= (PARODD | PARENB);
					options.c_iflag |= INPCK;
					break;
			 case 'e':
			 case 'E'://设置为偶校验
					options.c_cflag |= PARENB;
					options.c_cflag &= ~PARODD;
					options.c_iflag |= INPCK;
					break;
			 case 's':
			 case 'S': //设置为空格
					options.c_cflag &= ~PARENB;
					options.c_cflag &= ~CSTOPB;
					break;
			default:
					fprintf(stderr,"Unsupported parity/n");
					return (FALSE);
		}
		// 设置停止位
		switch (stopbits)
		{
			case 1:
					options.c_cflag &= ~CSTOPB;
					break;
			case 2:
					options.c_cflag |= CSTOPB;
					break;
			default:
					fprintf(stderr,"Unsupported stop bits/n");
					return (FALSE);
		}
		//修改输出模式，原始数据输出
		options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  //Input
		options.c_oflag  &= ~OPOST;   //Raw mode output		
		options.c_cc[VTIME] = 1; 	//设置等待时间,读取一个字符等待1*(1/10)s
		options.c_cc[VMIN] = 1; 	//设置最小接收字符,读取字符的最少个数为1
		tcflush(fd,TCIFLUSH);	//如果发生数据溢出，接收数据，但是不再读取
		if (tcsetattr(fd,TCSANOW,&options) != 0){	//激活配置 (将修改后的termios数据设置到串口中）
			perror("serial port set error!\n");
			return (FALSE);
		}
		return (TRUE);
}
/*******************************************************************
* 名称：          serialPortRead
* 功能：          接收串口数据
* 入口参数：      fd          :文件描述符
*                 rcv_buf     :接收串口中数据存入rcv_buf缓冲区中
*                 data_len    :读取数据的长度
* 出口参数：      正确返回读取的字符数，错误返回0
*******************************************************************/
int serialPortRead(int fd, char *rcv_buf,int data_len)
{
		int len,fs_sel;
		fd_set fs_read;
		struct timeval time;
		FD_ZERO(&fs_read);
		FD_SET(fd,&fs_read);
		time.tv_sec = 10;
		time.tv_usec = 0;
		//使用select实现串口的多路通信
		fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
		if(fs_sel){
							len = read(fd,rcv_buf,data_len);
							return len;
			 }else{
							return FALSE;
			 }
}
/*****************************************************************
* 名称：           serialPortClose
* 功能：           关闭串口
* 入口参数：       fd : 文件描述符
* 出口参数：       return 1 when run well else return 0.
*******************************************************************/
int serialPortClose(int fd){
	if(close(fd)){      //return 0 when succeed,return -1 when failed.
		return FALSE;
	}else{
		return TRUE;
	}
	perror("close serial port");
}