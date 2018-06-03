#ifndef SERIALPORT_H
#define SERIALPORT_H

//        flow_ctrl:
//        case 0 ://不使用流控制
//        case 1 ://使用硬件流控制
//        case 2 ://使用软件流控制
//        databits:
//        case 5:
//        case 6:
//        case 7:
//        case 8:
//        parity:
//        case 'n':
//        case 'N': //无奇偶校验位。
//        case 'o':
//        case 'O'://设置为奇校验
//        case 'e':
//        case 'E'://设置为偶校验
//        case 's':
//        case 'S': //设置为空格
//        stopbits:
//        case 1:
//        case 2:


class serialPort
{
public:
    serialPort(int baudrate,int flow_ctrl,int databits,int stopbits,int parity);
    ~serialPort();
    bool serialPortOpen(const char* port);
    bool  serialPortSet();
    int serialPortRead(char* rcv_buf,int data_len);
    bool serialPortClose();
public:
    int cBaudrate;//波特率
    int cFlow_ctrl;//流控制
    int cDatabits;//数据位
    int cStopbits;//停止位
    int cParity; //校验位
    int cFd;//文件描述符
};

#endif // SERIALPORT_H
