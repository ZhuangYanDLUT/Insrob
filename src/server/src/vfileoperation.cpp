#include "server/vfileoperation.h"

VFileOperation::VFileOperation()
{

}

VFileOperation::~VFileOperation()
{

}

void VFileOperation::fileSavePCxyzi(const string path, std::vector<VPointI>&cloud)
{    
    ofstream outfile(path, std::ios::out | std::ios::app | std::ios::binary);
    if(!outfile.is_open())
        return;
    for(int i = 0; i < cloud.size(); ++i)
        outfile.write((char*)&cloud[i], sizeof(VPointI));
    outfile.close();
}


void VFileOperation::fileSavePose(const string path, std::vector<VPose>&poses)
{
    ofstream outfile(path, std::ios::out |std::ios::app | std::ios::binary);
    if(!outfile.is_open())
        return;
    for(int i = 0; i < poses.size(); ++i)
        outfile.write((char*)&poses[i],sizeof(VPose));
    outfile.close();
}

void VFileOperation::fileSavePoseStamped(const string path, std::vector<VPoseStamped>&poses)
{
	//写二进制文件
    ofstream outfile(path, std::ios::out |std::ios::app | std::ios::binary);
    if(!outfile.is_open())
        return;
    for(int i = 0; i < poses.size(); ++i)
        outfile.write((char*)&poses[i],sizeof(VPoseStamped));
    outfile.close();
	
	//写Ascii码文件
	/*ofstream outfile(path, std::ios::out |std::ios::app);
    if(!outfile.is_open())
        return;
	
	outfile<<std::fixed;  //避免用科学计数法写文件
    outfile.precision(4);
    for(int i = 0; i < poses.size(); ++i)
	{
			outfile<<poses[i].timestamp<<" "<<poses[i].x<<" "<<poses[i].y<<" "<<poses[i].z
			<<" "<<poses[i].yaw<<" "<<poses[i].pitch<<" "<<poses[i].roll
			<<" "<<poses[i].i<<" "<<poses[i].j<<" "<<poses[i].k<<" "<<poses[i].w<<endl;
	}
    outfile.close();*/
	
	
	//写简化版Ascii码文件，时间戳格式为2007-01-15 15:43:21.237
	/*ofstream outfile(path, std::ios::out |std::ios::app);
    if(!outfile.is_open())
        return;
	
    time_t t;  //秒时间
    tm* local; //本地时间
    char buf[80]= {0};
    char buf2[16]= {0};
    char buf1[128]= {0};

    outfile<<std::fixed;
    outfile.precision(4);
    for(int i = 0; i < poses.size(); ++i)
    {
        t = time_t(poses[i].timestamp);
        local = localtime(&t); //转为本地时间
        strftime(buf, 64, "%Y-%m-%d %H:%M:%S", local);
        sprintf(buf2, "%.3f", poses[i].timestamp-(long long)poses[i].timestamp);
        //sprintf(buf1, "%s.%s", buf+11, buf2+2);
		sprintf(buf1, "%s.%s", buf, buf2+2);

        outfile<<buf1<<" "<<poses[i].x<<" "<<poses[i].y<<" "<<poses[i].z
            <<" "<<poses[i].yaw<<" "<<poses[i].pitch<<" "<<poses[i].roll<<endl;
    }
    outfile.close();*/
}

void VFileOperation::saveInfoTXT(const string path, string time, bool isSaveNavMap)
{
    ofstream outfile(path, std::ios::out);
    
    if(!outfile.is_open())
	return;
    cout<<"isSaveNavMap::"<<isSaveNavMap<<endl;
    if(isSaveNavMap)
    {
	outfile<<"isSaveNavMap"<<endl;
	outfile<<time;
    }
    else
    {
	outfile<<"NoNavMap"<<endl;
	outfile<<time;
    }
    outfile.close();
}

