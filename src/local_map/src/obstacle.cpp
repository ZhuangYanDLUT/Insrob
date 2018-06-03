#include <local_map/obstacle.h>
/*****************************************
**识别根据栅格特征计算障碍物的速度和方向
******************************************/
int total_index=1;          //卡曼滤波对象的起始id 
map<int,Obstacle_Feature>hisObs_feature;//历史障碍物索引 

void rec_obstacle(vector<map<float,PointData> >&_pointcloud,std::vector<GridData>&mapData,Map_info mapinfo,My_Pose pose)
{
	map<int,Obstacle_Feature>Obs_feature;//障碍物索引
	Obs_Segmentation(mapData,mapinfo,Obs_feature);//得到分割后的障碍物结果,并对结果进行过滤,删除体积过小或过大的物体

//std::cout<<">>>当前帧障碍数: "<<Obs_feature.size()<<" 历史帧障碍数: "<<hisObs_feature.size()<<std::endl;
	if(Obs_feature.size()!=0)
	{
		Obs_Classification(Obs_feature,_pointcloud);//对分割好的障碍物进行分类
		Obs_match(pose,Obs_feature,hisObs_feature);
	}
	else//当前时刻未检测到障碍
	{
		ClearKaman();
	}
	hisObs_feature=Obs_feature;//更新历史障碍物

//std::cout<<"-------------------------------------------"<<std::endl;

}


/*
* 匹配当前帧和历史帧的障碍
* 输入:	机器人位置：My_Pose pose
*		当前帧障碍物的特征：map<int,Obstacle_Feature>&Obs_feature
*		历史帧障碍物的特征：map<int,Obstacle_Feature>&hisObs_feature
* 输出：更新当前帧障碍物的特征中的：	float speed			//14、运动速度
*									float angle;		//15、运动方向
*									int K_index;		//对应的卡曼滤波对象id
*
*/

void Obs_match(My_Pose pose,map<int,Obstacle_Feature>&Obs_feature,map<int,Obstacle_Feature>&hisObs_feature)
{
	float r=Deal_r;//障碍物进行跟踪半径,超处半径认为超出机器人视野
	vector<My_Person> predict_pose;	//用于接收卡曼滤波的结果	
	PredictPose(pose,r,predict_pose);//接收卡曼滤波的结果即卡曼滤波预测当前状态下的障碍物集合
	
	for(map<int,Obstacle_Feature>::iterator it=hisObs_feature.begin();it!=hisObs_feature.end();it++)//遍历历史帧所有障碍
	{
		for(int i=0;i<predict_pose.size();i++)//遍历卡曼滤波预测的障碍物
		{
			if((*it).second.K_index==predict_pose[i].index)//找到历史帧障碍的预测结果，卡曼预测的每个历史障碍物在当前时刻位置
			{
				(*it).second.K_flag=true;//更新标志位			
				(*it).second.Kpred_x=predict_pose[i].P.x;//更新预测结果		
				(*it).second.Kpred_y=predict_pose[i].P.y;
				/*根据历史位置和卡曼滤波的预测位置计算卡曼滤波的速度和方向*/
				float his_x=(*it).second.top_p.global_x;
				float his_y=(*it).second.top_p.global_y;
				float pre_x=predict_pose[i].P.x;	
				float pre_y=predict_pose[i].P.y;	
				float dis_2add=(his_y-pre_y)*(his_y-pre_y)+(his_x-pre_x)*(his_x-pre_x);//增加的距离的平方和
				//计算速度，（卡曼预测的瞬时速度）
				(*it).second.speed=sqrt(dis_2add)*time_frequency;
							
			}
		}
		
	}

	//更新多线程参数变量
	Draw_object( pose, hisObs_feature);
	/*匈牙利匹配 [卡曼滤波预测] 和 [当前测量出的障碍]*/
	vector< vector<double> > costMatrix;//距离数组;
	//计算当前障碍特征，与卡曼预测障碍特征间距离的损失数组，即计算Obs_feature与hisObs_feature间的特征距离
	Cal_costMatrix(costMatrix,hisObs_feature,Obs_feature);

	HungarianAlgorithm HungAlgo;//建立匈牙利算法对象
	vector<int> assignment;//用于接收匈牙利匹配结果，如果为-1，则为新障碍
	//得到匈牙利匹配结果，cost可以作为匹配是否成功的标志,如果误差过大，用暴力匹配方法
	double cost = HungAlgo.Solve(costMatrix, assignment);


	/*根据历史障碍物更新目标,将原来的障碍的位姿进行更新*/
	vector<My_Person> updata_pose;	
	int obs_i=0;
	for(map<int,Obstacle_Feature>::iterator it=Obs_feature.begin();it!=Obs_feature.end();it++)
	{
		int hisobs_i=assignment[obs_i];
//std::cout<<"匈牙利匹配对应关系："<<obs_i<<"->"<<hisobs_i<<std::endl;
		if(hisobs_i!=-1)//当前障碍和过去障碍通过匈牙利匹配成功关联
		{
			map<int,Obstacle_Feature>::iterator hisit=hisObs_feature.begin();
			int tmp_i=hisobs_i;
			while(tmp_i)
			{
				tmp_i--;
				hisit++;//不断递增，使之指向历史障碍
//std::cout<<"tmp_i="<<tmp_i<<" "<<(*hisit).second.K_index<<" "<<std::endl;
			}
			My_Person now_p;	
			now_p.index=(*hisit).second.K_index;//得到历史障碍的id，当前障碍和历史障碍成功关联
			now_p.P.x=(*it).second.top_p.global_x;
			now_p.P.y=(*it).second.top_p.global_y;
			(*it).second.K_index=(*hisit).second.K_index;
/******************************************************/
/*打印障碍*/
float tmp_now_x=(*it).second.top_p.global_x;
float tmp_now_y=(*it).second.top_p.global_y;
float tmp_his_x=(*hisit).second.top_p.global_x;
float tmp_his_y=(*hisit).second.top_p.global_y;
float tmp_pre_x=(*hisit).second.Kpred_x;
float tmp_pre_y=(*hisit).second.Kpred_y;
/*
std::cout
<<std::endl
<<"#障碍物: "<<now_p.index<<" 匹配成功!"
<<" 栅格数: "<<(*it).second.grid_num
<<" 瞬时速度: "<<(*hisit).second.speed<<"m/s "
<<std::endl
<<"  当前坐标："<<" ["<<tmp_now_x<<","<<tmp_now_y<<"] "
<<"  历史坐标："<<" ["<<tmp_his_x<<","<<tmp_his_y<<"] "
<<"  预测坐标："<<" ["<<tmp_pre_x<<","<<tmp_pre_y<<"] "
<<std::endl
<<"  当前->预测： "<<sqrt((tmp_now_x-tmp_pre_x)*(tmp_now_x-tmp_pre_x)+(tmp_now_y-tmp_pre_y)*(tmp_now_y-tmp_pre_y))
<<"  当前->历史： "<<sqrt((tmp_now_x-tmp_his_x)*(tmp_now_x-tmp_his_x)+(tmp_now_y-tmp_his_y)*(tmp_now_y-tmp_his_y))
<<"  预测->历史： "<<sqrt((tmp_pre_x-tmp_his_x)*(tmp_pre_x-tmp_his_x)+(tmp_pre_y-tmp_his_y)*(tmp_pre_y-tmp_his_y))
<<std::endl;
*/
/******************************************************/
			updata_pose.push_back(now_p);		
		}
		else//未能成功关联，则为新加入的障碍物，赋给它一个新id即total_index
		{
			My_Person now_p;
			now_p.index=total_index;
			now_p.P.x=(*it).second.top_p.global_x;
			now_p.P.y=(*it).second.top_p.global_y;
			updata_pose.push_back(now_p);
			(*it).second.K_index=total_index;
			total_index++;	
/*
std::cout
<<std::endl	
<<"障碍物: "<<now_p.index<<" 新障碍物!"	
<<" 栅格数: "<<(*it).second.grid_num
<<" 当前坐标："<<" ["<<now_p.P.x<<","<<now_p.P.y<<"] "
<<std::endl;
*/
		}
		obs_i++;
	}
	UpdataPose(updata_pose);//更新卡曼滤波

}


