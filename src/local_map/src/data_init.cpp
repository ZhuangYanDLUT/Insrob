#include <local_map/data_init.h>
/*
*对原始激光点云进行旋转平移，将其从激光坐标系转换到全局坐标系
*/
void rec_initdata(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,vector<map<float,PointData> >&_pointcloud,My_Pose _pose)
{
	if(Laser_Line_Num!=16)//按照16线激光进行映射,如果更换激光，需编写对应的角度对应关系
	{	
		std::cout<<"[data_init.cpp] error:Laser_Line_Num!=16"<<std::endl;
		return;
	}
	/*分行保存点云数据*/
	for(int i=0;i<cloud->points.size();i++)
	{
		//过滤无效点
		if(std::isnan(cloud->points[i].x)||std::isnan(cloud->points[i].y)||std::isnan(cloud->points[i].z))continue;
		if(cloud->points[i].x<0.2&&cloud->points[i].y<0.2)continue;

		//局部坐标
		PointData p;
		p.x=cloud->points[i].x;
		p.y=cloud->points[i].y;
		p.z=cloud->points[i].z;
		p.i=cloud->points[i].intensity;
		
		//计算激光点属于哪条激光线，line_id[0,15]
		int angle,line_id;
		
		if(Laser_Line_Num==16)
		{
			angle=int(atan2(p.z,sqrt(p.x*p.x+p.y*p.y))*2*180/3.1415926);
			if(angle>0)line_id=8+int(angle/4);
			else line_id=7-int((-angle)/4);
		}

		//计算激光点在水平方向上的夹角theta[0.0,360.0)
		float theta=atan2(p.y,p.x)*180/3.1415926+180;

		//全局坐标-机器人pose
		p.rela_x=p.x*cos(_pose.yaw)-p.y*sin(_pose.yaw);
		p.rela_y=p.x*sin(_pose.yaw)+p.y*cos(_pose.yaw);
		p.rela_z=p.z;

		//全局坐标
		p.global_x=p.x*cos(_pose.yaw)-p.y*sin(_pose.yaw)+_pose.x;
		p.global_y=p.x*sin(_pose.yaw)+p.y*cos(_pose.yaw)+_pose.y;
		p.global_z=p.z+Laser_Height_Value+_pose.z;

		//存储激光点
		_pointcloud[line_id][theta]=p;
	}

}
/*
*载入旧栅格特征
*/
void rec_oldmap(std::vector<GridData>& mapData,std::vector<GridData>& history_mapData,Map_info& mapinfo,My_Pose _pose)
{
	/*遍历所有栅格*/
	for(int i=0;i<mapinfo.map_width * mapinfo.map_height;i++)
	{
		
		if(history_mapData[i].Is_boundary==true)
		{

			float rela_x,rela_y;
			rela_x=history_mapData[i].boundary_p.global_x-_pose.x;
			rela_y=history_mapData[i].boundary_p.global_y-_pose.y;
			float dx_f, dy_f;
			dx_f=rela_x+mapinfo.map_resolution*mapinfo.map_width*0.5;
			dy_f=rela_y+mapinfo.map_resolution*mapinfo.map_height*0.5;
			int32_t row, col, index; 
			col = lround(dx_f / mapinfo.map_resolution);
			row = lround(dy_f / mapinfo.map_resolution);
			if(row>-1 && row<mapinfo.map_width && col>-1 && col<mapinfo.map_height) //在地图内
			{
				index = (row*mapinfo.map_width)+col;//栅格索引
			}
			else continue;
	
			mapData[index].boundary_p=history_mapData[i].boundary_p;
			mapData[index].Is_boundary=true;

		}
					
	}

}

/*
*
*/
void rec_savemap(std::vector<GridData>& mapData,std::vector<GridData>& history_mapData)
{
	history_mapData=mapData;
/*
	history_mapData.resize(mapData.size());
	for(int i=0;i<mapData.size();i++)
	{
		history_mapData[i].top_p=mapData[i].top_p;		
	 	history_mapData[i].boundary_p=mapData[i].boundary_p;

		history_mapData[i].Is_boundary=mapData[i].Is_boundary;
		history_mapData[i].Is_obstacle=mapData[i].Is_obstacle;
		history_mapData[i].Is_obstacle_dynamic=mapData[i].Is_obstacle_dynamic;

		for(int j=0;j<history_mapData[i].numline.size();j++)
		{
			history_mapData[i].numline[j]=mapData[i].numline[j];
		}
		
		history_mapData[i].numPoint=mapData[i].numPoint;
		history_mapData[i].numSurPoint=mapData[i].numSurPoint;

		history_mapData[i].top=mapData[i].top;
		history_mapData[i].top_intensity=mapData[i].top_intensity;
		history_mapData[i].bottom=mapData[i].bottom;

		history_mapData[i].ave_height=mapData[i].ave_height;
		history_mapData[i].ave_intensity=mapData[i].ave_intensity;

	}
*/
}



