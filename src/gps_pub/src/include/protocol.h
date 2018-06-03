#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
using namespace std;

string head;
string timeUTC;
string latitude;
string northORsouth;
string longitude;
string eastORwest;
string GPSstatus;
string satelliteNum;
string HDOP;
string altitude;
string altitudeUnit;
string geiod;
string geiodUnit;
string rtkAge;
string rtkID;
int	   BCC;
#define FALSE 0
#define TRUE  1
/*****************************************************************
名称：splitString
功能：分割字符串
入口参数:s: the string
		v: the vector
		c: the separator
出口参数：void
*******************************************************************/
void splitString(const std::string& str, std::vector<std::string>& vec, const std::string& separator){
	std::string::size_type pos1, pos2;
	pos2 = str.find(separator);
	pos1 = 0;
	while(std::string::npos != pos2){
		vec.push_back(str.substr(pos1, pos2-pos1));
		pos1 = pos2 + separator.size();
		pos2 = str.find(separator, pos1);
	}
	if(pos1 != str.length())
		vec.push_back(str.substr(pos1));
}
/*****************************************************************
* 名称：findPacketHead
* 功能：find packet head 
* 入口参数：requestData1 : the string
* 出口参数：true return packetHeadKey,false return 0
*******************************************************************/
int findPacketHead(string requestData){	//搜索包头
	std::string::size_type pos=requestData.find_first_of("$");
	if(pos==string::npos){
		return FALSE;
	}else{
		// cout << "find $" << endl;
		return TRUE;
	}
}
/*****************************************************************
* 名称：fetchAFrame
* 功能：取出一帧
* 入口参数：requestData1:the string
* 出口参数：void
*******************************************************************/
string fetchAFrame(string requestData){
	string str("");
	std::string::size_type pos=requestData.find("\n");
	if(pos!=string::npos){
		str = requestData.substr(requestData.find_first_of('$'),requestData.find('\n'));
		// cout << str.size() << endl;
		return str;
	}
}
/*****************************************************************
* 名称：bccCheck
* 功能：BCC校验（异或校验）
* 入口参数：requestData:the string
* 出口参数：true return 1,false return 0
*******************************************************************/
int bccCheck(string requestData){
	stringstream ss;
	int a = requestData[0];
	int i=1;
	for(;requestData[i]!='*';i++){
		a ^= requestData[i];
	}
	ss<<requestData[i+1]<<requestData[i+2];
	ss>>BCC;
	ss.clear();
	/*判断是否通过校验*/
	if(BCC==a){
		return TRUE;
	}else{
		cout<<"校验失败"<<endl;
		return FALSE;
	}
}		
/*****************************************************************
* 名称：extractData
* 功能：分割一帧数据
* 入口参数：requestData:the string
* 出口参数：void
*******************************************************************/			
void extractData(string requestData){
	vector <string> splitedStr;
	splitString(requestData,splitedStr,",");
	head =          splitedStr[0];
	timeUTC =       splitedStr[1];
	latitude =      splitedStr[2];
	northORsouth =  splitedStr[3];
	longitude =     splitedStr[4];
	eastORwest=     splitedStr[5];
	GPSstatus=      splitedStr[6];
	satelliteNum=   splitedStr[7];
	HDOP=           splitedStr[8];
	altitude=       splitedStr[9];
	altitudeUnit=   splitedStr[10];
	geiod=          splitedStr[11];
	geiodUnit=      splitedStr[12];
	rtkAge=         splitedStr[13];
	vector <string> splitedStr1;
	splitString(splitedStr[14],splitedStr1,"*");
	rtkID=          splitedStr1[0];
}