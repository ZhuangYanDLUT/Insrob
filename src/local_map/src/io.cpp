#include <local_map/io.h>
/*
* 输出激光点云数据到文本文件
*/
void outputData(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,int32_t Out_filenum)
{
	if(file_num<=Out_filenum)
		file_num++;
	else 
		return;

	//文件名
	std::string path_postfix;
	std::stringstream ss;
	ss<<file_num;
	ss>>path_postfix;
	const char* s_out=(path_postfix+".txt").c_str();
	std::cout<<"out_file"<<s_out<<std::endl;
	std::ofstream out;
	out.open(s_out);

	float x, y, z, intensity;
	
	for(int i=0;i<cloud->points.size();i++)//对所有激光点处理
	{
		x=cloud->points[i].x;
		y=cloud->points[i].y;
		z=cloud->points[i].z;
		intensity=cloud->points[i].intensity;

		if(file_num<=Out_filenum)
		{
			if(std::isnan(x))out<<"0"<<" "<<"0"<<" "<<"0"<<" "<<"0"<<std::endl;//如果是无效值输出（0,0,0,0）
			else out<<x<< " "<<y<< " "<<z<< " "<<intensity<< " "<<std::endl;
		}

	}

	out.close();
}
