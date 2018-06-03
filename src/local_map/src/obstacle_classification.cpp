#include <local_map/obstacle_classification.h>
#include <local_map/io.h>

static int out_num=0;
void Obs_Classification(map<int,Obstacle_Feature>&Obs_feature,vector<map<float,PointData> >&_pointcloud)//对分割好的障碍物进行分类
{
	vector<vector<PointData> >Linepoint;
	for(int i=0;i<_pointcloud.size();i++)
	{
		vector<PointData>tmp_point;
		for(map<float,PointData>::iterator it=_pointcloud[i].begin();it!=_pointcloud[i].end();it++)
		{
			tmp_point.push_back((*it).second);
		}
		Linepoint.push_back(tmp_point);
	}
	/****************************************************************************************************************/


	int obs_id=0;
	for(map<int,Obstacle_Feature>::iterator it=Obs_feature.begin();it!=Obs_feature.end();it++)
	{
		obs_id++;
		//std::cout<<">>>obs_id: "<<obs_id<<"  numPoint: "<<(*it).second.numPoint<<std::endl;
		if((*it).second.numPoint>80)//示例
		{
			(*it).second.Is_human=false;		//不是行人
		}
	}
	
	data_out(Obs_feature,Linepoint);


	/****************************************************************************************************************/

	for(map<int,Obstacle_Feature>::iterator it=Obs_feature.begin();it!=Obs_feature.end();)//滤除不是行人的障碍
	{
		if( (*it).second.Is_human==false )	//滤除
		{
			Obs_feature.erase(it++);
		}	
		else
		{
			++it;
		}
	}

}

void data_out(map<int,Obstacle_Feature>&Obs_feature,vector<vector<PointData> >&Linepoint)
{
	//文件名
	out_num++;
	std::string path_postfix;
	std::stringstream ss;
	ss<<out_num;
	ss>>path_postfix;
	const char* s_out=(path_postfix+"_.txt").c_str();
	std::ofstream out;
	//std::cout<<"data_out "<<s_out<<endl;
	out.open(s_out);
	float x, y, z, intensity;
	int obs_id=-1;
	for(map<int,Obstacle_Feature>::iterator it=Obs_feature.begin();it!=Obs_feature.end();it++)
	{
		obs_id++;
		if((*it).second.Is_human==true)//是行人
		{
			//输出这个障碍物每条激光线上的激光点
			
			for(int i=0;i<(*it).second.point_id.size();i++)
			{

				for(int j=0;j<(*it).second.point_id[i].size();j++)
				{
					int _id=(*it).second.point_id[i][j];
					PointData p=Linepoint[i][_id];
					//第obs_id个障碍物、第i行、的坐标（xyz）
					out<<obs_id<<" "<<i<<" "<<p.x<< " "<<p.y<< " "<<p.z<<std::endl;
				}

			}
			
		}
	}
	out.close();

}




