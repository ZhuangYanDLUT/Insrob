#include <local_map/map_builder.h>
#include <pthread.h>
boost::mutex  mtx_pose,mtx_map;

namespace local_map {

MapBuilder::MapBuilder(unsigned short width, unsigned short height, double resolution)
{
    map_.header.frame_id = "/camera_init";		//Rviz显示的frame_id
    map_.info.width = width;            //宽度方向上栅格地图个数
    map_.info.height = height;          //高度方向上栅格地图个数
    map_.info.resolution = resolution;  //栅格边长
    map_.data.assign(width * height, labels[0]);    //初始化栅格地图标签为0 
 	map_.info.origin.position.x = -static_cast<double>(width) / 2 * resolution; //0号栅格（高度和宽度方向上的最小栅格）左下角坐标（真实值）
    map_.info.origin.position.y = -static_cast<double>(height) / 2 * resolution;
	//map_.info.origin.position.x = 0;
	//map_.info.origin.position.y = 0;
	history_mapData.assign(  width * height, GridData());
	/*地图参数*/
	mapinfo.map_position_x=	map_.info.origin.position.x;
	mapinfo.map_position_y=	map_.info.origin.position.y;
	mapinfo.map_width = map_.info.width;           
    mapinfo.map_height= map_.info.height;         
    mapinfo.map_resolution= map_.info.resolution;
 
	/*动态障碍可视化线程*/
	//pthread_t tid;
    //pthread_create(&tid,NULL,Show_object,NULL);
}

MapBuilder::~MapBuilder(){}
/*
* 入口函数
*/
void MapBuilder::updateMap(sensor_msgs::PointCloud2 &pointClouds_msg)
{
	/*初始化*/
	mapData.assign( map_.info.width *  map_.info.height, GridData());
	/*数据类型转换*/
	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::moveFromROSMsg(pointClouds_msg,*pcl_cloud);
	/*S0对激光数据进行旋转平移,得到全局坐标系下的激光点*/
	vector<map<float,PointData> >_pointcloud(Laser_Line_Num);//分行存储的激光点云对象
	rec_initdata(pcl_cloud,_pointcloud,_pose);//[data_init.cpp]
	/*******************处理激光点*****start*****************/
	///S1 载入累计障碍点（静态、动态障碍，道路边缘）
	rec_oldmap(mapData,history_mapData,mapinfo, _pose);//[data_init.cpp]
	///S2 提取简单的栅格特征,并判断障碍
	rec_gridfeature(_pointcloud,mapinfo,mapData);//[grid_feature.cpp]
	///S3 道路边缘检测
	rec_roadboundary(_pointcloud,mapinfo,mapData);//[road_boundary.cpp]
	///S4 基于栅格特征识别障碍（动态）
	//rec_obstacle(_pointcloud,mapData,mapinfo,_pose);	
	///S5 更新累计障碍点（道路边缘）
	rec_savemap(mapData,history_mapData);
	/*******************处理激光点*****end  *****************/
	
	/*******************更新地图**********************/
	/*根据栅格特征更新地图*/
	mtx_map.lock();
	generate_map();
	mtx_map.unlock();

	//数据输出
	//if(Is_output)outputData(pcl_cloud,Out_filenum);//输出单帧数据在dev文件夹

}

void MapBuilder::setPoseAndHeading(float x, float y, float z, float angle)//更新机器人位置
{
    mtx_pose.lock();
    _pose.x = x;
    _pose.y = y;
    _pose.z = z;
   	_pose.yaw = angle;
	map_.info.origin.position.x = x-static_cast<double>(map_.info.width) / 2 * map_.info.resolution; //更新地图在Rviz中的位置
    map_.info.origin.position.y = y-static_cast<double>(map_.info.height) / 2 * map_.info.resolution;
    mtx_pose.unlock();
}

void MapBuilder::generate_map()
{
	map_.data.assign( map_.info.width *  map_.info.height, labels[0]);    
	for(int i=0;i<map_.info.width * map_.info.height;i++)
	{
		if(mapData[i].Is_obstacle==true)
		{
			map_.data[i]=labels[1];//障碍栅格
		}
		if(mapData[i].Is_boundary==true)
		{
			map_.data[i]=labels[3];//道路边缘栅格
		}
		if(mapData[i].Is_obstacle_dynamic==true)
		{
			//map_.data[i]=labels[2];//动态障碍
		}	
		
	}
}

nav_msgs::OccupancyGrid MapBuilder::getMap()
{
	boost::mutex::scoped_lock lock(mtx_map);
	
    return map_;
}

}//end of namespace
/************************/


