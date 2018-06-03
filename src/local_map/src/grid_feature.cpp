#include <local_map/grid_feature.h>
/*
* 计算简单的栅格特征
*/

void rec_gridfeature(vector<map<float,PointData> >&pointcloud,Map_info mapinfo,std::vector<GridData> &mapData)
{
	const float Obs_height=threshold_height;
	/*遍历所有激光数据*/
	for(int i=0;i<Laser_Line_Num;i++)//遍历所有激光数据
	{
		int point_id=-1;
		for(map<float,PointData>::iterator it=pointcloud[i].begin();it!=pointcloud[i].end();it++)
		{
			point_id++;
			/*获取栅格索引*/
			PointData p=(*it).second;
			float dx_f, dy_f;
			dx_f=p.rela_x+mapinfo.map_resolution*mapinfo.map_width*0.5;
			dy_f=p.rela_y+mapinfo.map_resolution*mapinfo.map_height*0.5;
			int32_t row, col, index; 
			col = lround(dx_f / mapinfo.map_resolution);
			row = lround(dy_f / mapinfo.map_resolution);
			if(row>-1 && row<mapinfo.map_width && col>-1 && col<mapinfo.map_height) //在地图内
			{
				index = (row*mapinfo.map_width)+col;//栅格索引
			}
			else continue;

			/*栅格属性*/
			mapData[index].numPoint++;
			mapData[index].numline[i]++;
			mapData[index].point_id[i].push_back(point_id);
			mapData[index].ave_height+=p.rela_z;
			mapData[index].ave_intensity+=p.i;
			if(p.rela_z>mapData[index].top)
			{
				mapData[index].top=p.rela_z;
				mapData[index].top_p=p;
				mapData[index].top_intensity=p.i;

				
			}
			if(p.rela_z<mapData[index].bottom)
			{
				mapData[index].bottom=p.rela_z;
			}

			/*对下激光:[0-7]行激光线进行处理*/
			if(i>=0&&i<=Laser_Line_Num/2-1)
			{
				
			}

			/*对上激光:[9-15]行激光线进行处理*/
			else if(i>=Laser_Line_Num/2+1&&i<Laser_Line_Num)
			{
		
			}

			/*对水平激光:[7-8]行激光线进行处理*/
			else if(i==Laser_Line_Num/2-1||i==Laser_Line_Num/2)
			{
				mapData[index].numSurPoint++;
			}
		}
	}

	/*遍历所有栅格*/
	int nei8[16]={	-1,-1, 0,-1, 1,-1,
					-1, 0,		 1, 0,
					-1, 1, 0, 1, 1, 1};
	for(int i=0;i<mapinfo.map_width * mapinfo.map_height;i++)
	{
		if(mapData[i].numPoint==0)continue;//空栅格
		mapData[i].ave_height=mapData[i].ave_height/mapData[i].numPoint;
		mapData[i].ave_intensity=mapData[i].ave_intensity/mapData[i].numPoint;
		if(mapData[i].top-mapData[i].bottom>Obs_height||mapData[i].numSurPoint>0)
		{
			mapData[i].Is_obstacle=true;
			//8邻域膨胀
			/*
			for(int t=0;t<8;t++)
			{
				int32_t row, col, index; 
				col = i % ((int)(mapinfo.map_width))+nei8[2*t];
				row = i / mapinfo.map_width+nei8[2*t+1];

				if(row>-1 && row<mapinfo.map_width && col>-1 && col<mapinfo.map_height) //在地图内
				{
					index = (row*mapinfo.map_width)+col;//栅格索引
					mapData[index].Is_obstacle=true;
				}
			}
			*/
		}
				
	}

}
