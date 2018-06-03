#include <local_map/road_boundary.h>
/*
* 判断到路边缘
* 方法:同一条激光线上连续[]个激光点在z方向的切线分量满足[]
*/

void rec_roadboundary(vector<map<float,PointData> >&pointcloud,Map_info mapinfo,std::vector<GridData> &mapData)
{
	using namespace std;
	const int n_line=5;	//用n_line条激光线上的数据识别道牙子
	const int n_window=8;//邻域窗口大小	
	//判断是否为道牙子的参数
	const float val_dz = 0.10;//切向量z方向的分量-越大越严苛
	const float val_num = 20;//最少连续点数-越大越严苛
	const float val_dis_maxz= 0.15;//z方向高度差（最大范围）-越小越严苛
	const float val_dis_minz = 0.05;//z方向高度差（最小范围）-越大越严苛
	const float val_maxz = -Laser_Height_Value+0.15;//最高点（道牙子最高点相对于地面的最高高度）
	const float val_neighbourdis = 0.1;//相邻点的分割条件距离

/*************************************************************************/
	vector<vector<PointData> >Points;//识别道路边缘所需要的激光点
	Points.resize(n_line);
	for(int i=0;i<n_line;i++)//初始化：Points
	{
		Points[i].resize(pointcloud[i].size()+2 * n_window);
	}

	for(int i=0;i<n_line;i++)
	{
		int j=0;
		for(map<float,PointData>::iterator it=pointcloud[i].begin();it!=pointcloud[i].end();it++)
		{
			Points[i][n_window+j]=(*it).second;
			j++;
		}	
	}

	//补全两端：[n_window][pointcloud[j].size()][n_window]
	for (int i = 0; i < n_window; i++)
	{
		for (int j = 0; j < n_line; j++)
		{
			Points[j][i] = Points[j][pointcloud[j].size() + i];
			Points[j][n_window + i + pointcloud[j].size()] = Points[j][n_window + i];
		}
	}

	vector<float> t_sample;
	vector<float> t_discrete;
	vector<vector<float> >az_discrete;	//切向量在z方向的分量
	vector<vector<float> >dis_neipoints;//相邻点距离
	
	az_discrete.resize(n_line);
	dis_neipoints.resize(n_line);
	for(int i=0;i<n_line;i++)
	{
		az_discrete[i].resize(pointcloud[i].size());
		dis_neipoints[i].resize(pointcloud[i].size());
	}

	for (int l = 0; l < n_line; l++)//对每条激光线进行处理
	{
		int numPoints= Points[l].size();//当前行总点数（补全后）
		int n_eashline=pointcloud[l].size();
		t_discrete.resize(numPoints);	
		t_sample.resize(numPoints);		
		float dx_discrete,dy_discrete,dz_discrete,x_st,y_st,z_st;
		dx_discrete=0;dy_discrete =0;dz_discrete=0;
		float total_st=0;
		for (int i = 1; i<numPoints; i++)
		{
			x_st = (Points[l][i].x - Points[l][i - 1].x);	
			y_st = (Points[l][i].y - Points[l][i - 1].y);
			z_st = (Points[l][i].z - Points[l][i - 1].z);
			float tmp_st=sqrt((x_st*x_st) + (y_st*y_st) + (z_st*z_st));	
			if(numPoints-n_window>=0&&numPoints-n_window<n_eashline)
				dis_neipoints[l][numPoints-n_window]=tmp_st;
			total_st = total_st+tmp_st;
			t_discrete[i] = total_st;
		}

		for (int i = 0; i<numPoints; i++)
		{
			t_sample[i] = (t_discrete[i] / total_st);
		}
		
		for (int i = n_window; i< numPoints-n_window; i++)
		{
			int n_start = i-n_window;
			int n_end = i + n_window;
			float sum_numerator_x = 0, sum_denominator_x = 0;
			float sum_numerator_y = 0, sum_denominator_y = 0;
			float sum_numerator_z = 0, sum_denominator_z = 0;

			for (int j = n_start; j <= n_end; j++)
			{
				sum_numerator_x += (t_sample[j] - t_sample[i])*(Points[l][j].x - Points[l][i].x);
				sum_denominator_x += pow((t_sample[j] - t_sample[i]), 2);

				sum_numerator_y += (t_sample[j] - t_sample[i])*(Points[l][j].y - Points[l][i].y);
				sum_denominator_y += pow((t_sample[j] - t_sample[i]), 2);

				sum_numerator_z += (t_sample[j] - t_sample[i])*(Points[l][j].z - Points[l][i].z);
				sum_denominator_z += pow((t_sample[j] - t_sample[i]), 2);
			}

			dx_discrete = (sum_numerator_x / sum_denominator_x);
			dy_discrete = (sum_numerator_y / sum_denominator_y);
			dz_discrete = (sum_numerator_z / sum_denominator_z);
			float tmpdr_discrete = (sqrt(pow(dx_discrete, 2) + pow(dy_discrete, 2) + pow(dz_discrete, 2)));
			az_discrete[l][i- n_window] = dz_discrete / tmpdr_discrete;	
		}
	}

	for (int l = 0; l < n_line; l++)//对于每条激光线
	{
		vector<PointData>boundary_data;
		vector<float>boundary_val(2);//{min_z,max_z}
		int n_eashline =pointcloud[l].size();
		for (int i = 0; i < n_eashline; i++)
		{
			if (fabs(az_discrete[l][i]) <= val_dz ||dis_neipoints[l][i]>val_neighbourdis|| i == (n_eashline - 1))//分割条件
			{
				if (boundary_data.size() == 0)continue;
				if (boundary_data.size() < val_num) {}
				else
				{
					if (boundary_val[1] > val_maxz || boundary_val[1]-boundary_val[0]< val_dis_minz || boundary_val[1]-boundary_val[0] > val_dis_maxz ) //过滤非道牙子点		
					{
					}
					else
					{						
						for (int t = 0; t < boundary_data.size(); t++)//得到道牙子点
						{
							//向地图中投影
							float dx_f, dy_f;
							dx_f=boundary_data[t].rela_x+mapinfo.map_resolution*mapinfo.map_width*0.5;
							dy_f=boundary_data[t].rela_y+mapinfo.map_resolution*mapinfo.map_height*0.5;
							int32_t row, col, index;
							col = lround(dx_f / mapinfo.map_resolution);
							row = lround(dy_f / mapinfo.map_resolution);
							
							if(row>-1 && row<mapinfo.map_width && col>-1 && col<mapinfo.map_height) //在地图内
							{
								index = (row*mapinfo.map_width)+col;//栅格索引
								mapData[index].boundaryPoint++;
								//如果只是一个普通障碍不进行累计
								mapData[index].boundary_p=boundary_data[t];////保存一个道牙子的全局坐标，用于累计
							}
						}
					}
				}
				boundary_data.clear();
				boundary_val.resize(2);
			}
			else //存储疑似道牙子簇
			{
				boundary_data.push_back(Points[l][i]);
				if(boundary_data.size()!=1)
				{
					if(boundary_val[0]>Points[l][i].z)boundary_val[0]=Points[l][i].z;
					if(boundary_val[1]<Points[l][i].z)boundary_val[1]=Points[l][i].z;
				}
				else
				{
					boundary_val[0]=Points[l][i].z;
					boundary_val[1]=Points[l][i].z;
				}
			}
			
		}//end of "for (int i = 0; i < n_eashline; i++)"
			
	}//end of "for (int l = 0; l < n_line; l++)"

	/*遍历所有栅格中存储的道牙子点个数，更新道牙子属性*/
	int nei24[48]={	-2,-2, -1,-2,  0,-2, +1,-2, +2,-2, 
					-2,-1, -1,-1,  0,-1, +1,-1, +2,-1, 
					-2, 0, -1, 0,        +1, 0, +2, 0, 
					-2,+1, -1,+1,  0,+1, +1,+1, +2,+1, 
					-2,+2, -1,+2,  0,+2, +1,+2, +2,+2};
	for(int i=0;i<mapinfo.map_width * mapinfo.map_height;i++)
	{
		if(mapData[i].numPoint==0)continue;//空栅格
		if(mapData[i].Is_obstacle==true)continue;//障碍栅格不再标记为道牙子

		if(mapData[i].boundaryPoint>0)
		{
			bool flag_obs=true;
			for(int t=0;t<24;t++)
			{
				int32_t row, col, index; 
				col = i % mapinfo.map_width+nei24[2*t];
				row = i / mapinfo.map_width+nei24[2*t+1];
				if(row>-1 && row<mapinfo.map_width && col>-1 && col<mapinfo.map_height) //在地图内
				{
					index = (row*mapinfo.map_width)+col;//栅格索引
					if(/*mapData[index].numSurPoint>0*/mapData[index].Is_obstacle==true)
					{
						flag_obs=false;
						break;
					}
				}
			}
			if(flag_obs==true)mapData[i].Is_boundary=true;
			
		}
				
	}
	
}




