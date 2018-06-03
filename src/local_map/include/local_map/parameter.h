#ifndef LOCAL_MAP_PARAMETER_H
#define LOCAL_MAP_PARAMETER_H

#include <vector>
#include <map>
using namespace std;
//激光参数
const int Laser_Line_Num=16;  //激光线数
const float Laser_Height_Value=0.90;  //激光对地高度
const int MAP_WIDTH=150;
const int MAP_HEIGHT=150;
/*位姿*/
struct My_Pose
{
	My_Pose():x(0),y(0),z(0),yaw(0),time(0){}
	float time;//时间单位秒
	float x,y,z;
	float yaw;//范围：[-PI,PI]
};

/*行人对象,用于卡曼滤波的跟踪*/

struct My_Person
{
	My_Pose P;
	int index;
};

/*每个激光数据点*/
struct PointData
{
	PointData():x(0),y(0),z(0),i(0),global_x(0),global_y(0),global_z(0),rela_x(0),rela_y(0),rela_z(0),line_id(-1){}
	/*所属激光线id*/
	int line_id;
	/*原始激光数据：以激光为原点，地面高度为-Laser_Height_Value*/
	float x,y,z;
	/*强度[0-255]*/
	int i;
	/*世界坐标系下的坐标*/
	float global_x,global_y,global_z;//地面高度为0
	/*用于地图投影的坐标*/
	float rela_x,rela_y,rela_z;//地面高度为0
	
};

/*障碍特征*/
struct Obstacle_Feature//障碍栅格聚类特征
{
	Obstacle_Feature():numPoint(0),numSurPoint(0),grid_num(0),top(-999),length(0),previous_id(-1),K_flag(false),Is_human(true)
	{	
		numline.resize(Laser_Line_Num, 0);	
		point_id.resize(Laser_Line_Num);
	}
	//特征：位置,形状[宽度、高度],姿态
	//->位置（[8]、[9]）& 姿态（[4]中上激光/[7]):点数反映了弧长关系

	PointData top_p;		//存储障碍物的最高点的全局坐标,即障碍物的位置

	vector<int>Grid_cluster;//0、障碍包含的栅格的(grid_id)
 	int numPoint;			//1、[点数特征]包含的总点数
	int numSurPoint;		//2、[角度特征]水平方向上检测到的激光点
	int grid_num;			//3、[形状特征]包含栅格的数量
	vector<int>numline;		//4、[形状特征]每条激光线扫描到的点数
	vector<vector<int> >point_id;		//每条激光线上包含激光点的ID

	float top;				//5、[形状特征]最高点的高度
	//int max_line;			//6、[形状特征]最高点对应的激光线号
	float ave_height;		//7、[形状特征]栅格内的平均高度
	float top_intensity;	//8、 [纹理特征]栅格内点云高程最大值所对应激光点的强度
	float ave_intensity;	//9、 [纹理特征]栅格内的平均强度

	int length;				//10、[形状特征]栅格聚类外边边长
	//double distance;		//11、 [点数特征]聚类中心点栅格到激光器的距离
	int previous_id;		//12、[障碍关联]对应的历史障碍的中心栅格id
	int label;				//13、障碍物标签[0:未知，1：非行人，2：疑似行人]
	float speed;			//14、运动速度
	float angle;			//15、运动方向

	int K_index;			//对应的卡曼滤波对象id
	float Kpred_x;			//卡曼滤波预测的位置
	float Kpred_y;
	bool K_flag;			//卡曼标志位，即下一帧还会出现在地图中

	bool Is_human;		//是否为行人

};


/*栅格特征*/
struct GridData
{
    GridData():top(-999), bottom(999),  numPoint(0),numSurPoint(0),Is_boundary(false),Is_obstacle(false),Is_obstacle_dynamic(false),boundaryPoint(0),flag_search(false)
	{
		numline.resize(Laser_Line_Num);
		point_id.resize(Laser_Line_Num);
	}

	/*点数特征*/
    int numPoint;			//栅格内包含的总点数
	int numSurPoint;		//栅格内包含水平激光（16线激光的中间两线）点数
	vector<int>numline;		//栅格内每条激光线包含的点数
	vector<vector<int> >point_id;		//栅格在每条激光线上包含激光点的ID

	/*高程特征*/
	float top;				//栅格内最高点(高度)
	float top_intensity;	//栅格内点云高程最大值所对应激光点的强度
    float bottom;			//栅格内最低点
	float ave_height;		//栅格内的平均高度
	float ave_intensity;	//栅格内的平均强度
	int boundaryPoint; 		//道牙子点的数目（备份的过程存在衰减）
	
	/*栅格标志位*/
	bool Is_boundary;		//是否为道路边缘（道牙子）[-1:误检测点，0:未检测，1:道牙子]
	bool Is_obstacle;		//是否为障碍
	bool Is_obstacle_dynamic;//是否为动态障碍

	bool flag_search;		//区域搜索标志位
	/*备份点*/
	PointData top_p;		//存储障碍物的最高点的全局坐标
	PointData boundary_p;	//存储一个道牙子点的全局坐标用于保存和恢复

};

/*地图信息*/
struct Map_info
{
	/*栅格地图左下角栅格的全局坐标*/
	float map_position_x;     
	float map_position_y;

	int map_height;	//地图高
	int map_width;	//地图宽
	float map_resolution;//地图分辨率
};


//地图相关参数
const unsigned char labels[4] = {1,120,180,240};       	//栅格标签 0可行，1障碍，2动态障碍，3道牙子
//const unsigned char labels[4] = {1,80,100,130};
//障碍参数
const float threshold_height=0.25;			//可通行高度（栅格内点的高度差）
const float mini_speed=0.5;					//速度大于0.5m/s的障碍认为是动态障碍
const int pic_size=720;						//可视化界面大小

//输出激光数据相关参数
static bool Is_output=true;
static int Out_filenum=100000;
static int file_num=0;


#endif  // LOCAL_MAP_PARAMETER_H
