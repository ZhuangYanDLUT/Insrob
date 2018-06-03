#include <local_map/obstacle_segmentation.h>
/*
* 根据水平激光在地图中的投影，将栅格地图分割出一个个独立的障碍物，并提取障碍物的特征
*/
void Obs_Segmentation(std::vector<GridData>& mapData,Map_info mapinfo,map<int,Obstacle_Feature>& Obs_feature)
{
	//八邻域
	int nei_8[16] = {-1,-1,0,-1,1,-1,-1,0,1,0,-1,1,0,1,1,1 };
	//将vector转为map
	map<int, GridData>Vec_feature;
	int total_grid_num=mapinfo.map_width * mapinfo.map_height;//栅格地图总数
	for(int i=0;i<total_grid_num;i++)
	{
		if(mapData[i].Is_obstacle==true)
		{
			Vec_feature[i]=mapData[i];
		}
	}
	//区域生长
	for (map<int, GridData>::iterator it = Vec_feature.begin(); it != Vec_feature.end(); it++)
	{
		if ((*it).second.flag_search == 1|| (*it).second.numSurPoint == 0)//被搜索过&没有被水平激光扫到
			continue;

		Obstacle_Feature of;
		int center_id;//障碍中心对应的栅格id[0~total_grid_num]
		vector<int> Grid_cluster;//存储聚类结果数组

		//区域生长
		vector<int>Vec_grid;//用于区域生长的临时数组
		Vec_grid.push_back((*it).first);
		Vec_feature[(*it).first].flag_search = true;


		while (Vec_grid.size())
		{
			int grid_search=Vec_grid[Vec_grid.size() - 1];
			Grid_cluster.push_back(grid_search);
			Vec_grid.pop_back();
			
			//int grid_id = Gridxy2Gridid(grid_search.x, grid_search.y);
			int grid_id=grid_search;
			//开始提取障碍物特征	

			of.Grid_cluster.push_back(grid_id);					//0:障碍包含的栅格的(grid_id)
			of.numPoint += Vec_feature[grid_id].numPoint;		//1:包含的总点数
			
			of.numSurPoint += Vec_feature[grid_id].numSurPoint;	//2:水平方向上检测到的激光点
			of.grid_num++;										//3:[形状特征]包含栅格的数量

			for (int nl = 0; nl < Laser_Line_Num; nl++)//每条线的点数
			{
				of.numline[nl] += Vec_feature[grid_id].numline[nl];//4:每条激光线扫描到的点数
				for(int nid=0;nid<Vec_feature[grid_id].point_id[nl].size();nid++)
				{
					of.point_id[nl].push_back(Vec_feature[grid_id].point_id[nl][nid]);
				}
			}

			if (of.top < Vec_feature[grid_id].top)
			{
				center_id = grid_id;
				of.top = Vec_feature[grid_id].top;				//5:[形状特征]最高点的高度
				//of.max_line = Vec_feature[grid_id].max_line;	//6:[形状特征]最高点对应的激光线号
				of.top_p=Vec_feature[grid_id].top_p;
			}
			
			for (int i = 0; i < 8; i++)//搜索8邻域栅格
			{
				int row, col, id; //邻域栅格id
				col = grid_id%mapinfo.map_width;
				row = grid_id/mapinfo.map_width;
				col += nei_8[2*i];
				row += nei_8[2*i+1];
				if(row>-1 && row<mapinfo.map_width && col>-1 && col<mapinfo.map_height) //在地图内
				{
					id = (row*mapinfo.map_width)+col;//栅格索引
					of.length++;
				}
				else 
				{
					continue;
				}
				if (Vec_feature.find(id) == Vec_feature.end())//栅格不存在
				{
					continue;
				}
				if (Vec_feature[id].flag_search == 1)//被搜索过
				{
					continue;
				}
				Vec_grid.push_back(id);
				Vec_feature[id].flag_search = 1;
			}
		}

		if (of.grid_num <= Min_obsnum)continue;//障碍物占据的栅格过少，剔除
		if (of.grid_num >= Max_obsnum)continue;//障碍物占据的栅格过多，剔除
		if (of.top >= 2)continue;//障碍物过高，剔除
		//mapData[center_id].Is_obstacle_dynamic=true;
		Obs_feature[center_id] = of;
		
	}
}

