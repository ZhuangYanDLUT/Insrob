#include <local_map/obstacle_view.h>

boost::mutex  mtx_object;

/*用于多线程显示的共享变量*/	
map<int,Obstacle_Feature> thread_feature;
My_Pose thread_pose;

/*
*更新多线程参数变量
*/
void Draw_object(My_Pose pose,map<int,Obstacle_Feature>&hisobs_feature)
{
	mtx_object.lock();
	thread_feature=hisobs_feature;
	thread_pose=pose;
	mtx_object.unlock();
}

/*
* 可视化的线程函数
*/

void* Show_object(void*)
{
	/*创建可视化窗口*/
	Mat image(720,720,CV_8UC3,Scalar(0)); 
	namedWindow("Track"); 
	int size=pic_size;
	
	/*
	char buf[256];  
    snprintf(buf,256,"Time:");  
    putText(image,buf,Point(10,30),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,0),1,4);  
	*/
	while(1)
	{
		image.setTo(Scalar(255,255,255,0)); 
		int obs_num=0;
		for(map<int,Obstacle_Feature>::iterator it=thread_feature.begin();it!=thread_feature.end();it++)//遍历历史帧所有障碍
		{
			if((*it).second.K_flag!=true)
				continue;

			//文本
			obs_num++;
			char buf[256];  
    		snprintf(buf,256,"Speed:%.2f",(*it).second.speed);  
    		putText(image,buf,Point(10,30*obs_num),CV_FONT_HERSHEY_SIMPLEX,1,
					Scalar(17*((*it).second.K_index*3)%255,17*((*it).second.K_index*5)%255,17*((*it).second.K_index*7)%255),1,4);  
			//预测点
			CvPoint center_pred;
			center_pred.x=(int)( ((*it).second.Kpred_x-thread_pose.x)*(size/15) +size/2);
			center_pred.y=size-(int)( ((*it).second.Kpred_y-thread_pose.y)*(size/15) +size/2);
			circle(image,center_pred,2,
					Scalar(17*((*it).second.K_index*3)%255,17*((*it).second.K_index*5)%255,17*((*it).second.K_index*7)%255),2);//圆形
			

			//历史点
			CvPoint center_his;
			center_his.x=(int)( ((*it).second.top_p.global_x-thread_pose.x)*(size/15) +size/2);
			center_his.y=size-(int)( ((*it).second.top_p.global_y-thread_pose.y)*(size/15) +size/2);
			circle(image,center_his,2,
					Scalar(17*((*it).second.K_index*3)%255,17*((*it).second.K_index*5)%255,17*((*it).second.K_index*7)%255),3);//方形

		}

		imshow("Track", image); 
	
		int code = (char)waitKey(100);
		if(code == 'q' || code == 'Q' )break;  
	}
	
	 
}



