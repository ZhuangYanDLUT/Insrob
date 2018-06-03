#include <local_map/obstacle_distance.h>
/*
* 计算障碍物之间的特征距离
*/
void Cal_costMatrix(vector< vector<double> >&costMatrix,map<int,Obstacle_Feature>hisObs_feature,map<int,Obstacle_Feature>Obs_feature)
{
	const float max_2euclidean=1;//最大平方距离
	//距离所占权重
	const float weight_euclidean=1;
	const float weight_linenum=0;

	int obs_num=-1;
	for(map<int,Obstacle_Feature>::iterator it=Obs_feature.begin();it!=Obs_feature.end();it++)//对于当前时刻的障碍物
	{
		obs_num++;
		int his_num=-1;
		vector<double>dis_cost;
		for(map<int,Obstacle_Feature>::iterator hisit=hisObs_feature.begin();hisit!=hisObs_feature.end();hisit++)
		{
			his_num++;
			float his_x,his_y,now_x,now_y;//历史帧和当前帧的全局坐标
			if(!(*hisit).second.K_flag)//卡曼滤波的估计值在地图之外
			{
				his_x=(*hisit).second.top_p.global_x;
				his_y=(*hisit).second.top_p.global_y;
				now_x=(*it).second.top_p.global_x;
				now_y=(*it).second.top_p.global_y;
			}
			else
			{
				his_x=(*hisit).second.Kpred_x;
				his_y=(*hisit).second.Kpred_y;
				now_x=(*it).second.top_p.global_x;
				now_y=(*it).second.top_p.global_y;
			}
			//欧氏距离的平方
			float euclidean_distance=((now_x-his_x)*(now_x-his_x)+(now_y-his_y)*(now_y-his_y));			

			//特征距离
			
			int linenum_distance;//每行激光线的点数差的绝对值
			for(int i=7;i<16;i++)//[7,8][9,10,11,12,13,14,15]考虑9条激光线
			{
				linenum_distance+=abs((*it).second.numline[i]-(*hisit).second.numline[i]);
			}

			float distance_cost=weight_euclidean*euclidean_distance+weight_linenum*linenum_distance;//特征加权和
			/*
			std::cout<<"当前障碍: "<<obs_num<<" ["<<now_x<<","<<now_y<<"]"<<"--->"
					<<" 历史障碍: "<<his_num<<" ["<<his_x<<","<<his_y<<"]"
					<<" 欧式距离: "<<euclidean_distance
					<<" 特征距离: "<<distance_cost
					<<std::endl;
			*/
			dis_cost.push_back( distance_cost);
		}
		costMatrix.push_back(dis_cost);
	}

}

