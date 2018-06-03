#include <local_map/obstacle_kaman.h>
                 
map<int,KalmanFilter>MKF;   //卡曼滤波对象  
/*
* 初始化卡曼滤波对象map
*/
void ClearKaman()
{
	MKF.clear();
}

/*
* 得到（指定区域中）所有目标的预测全局位姿
* 给定限制范围，根据过往目标输出满足条件的预测结果，范围过大则清除
* 输入机器人全局位姿：pose
* 获得预测的位姿 vector<PointData>&predict_pose
*/
void PredictPose(My_Pose pose,float r,vector<My_Person>&predict_pose)
{
	for(map<int,KalmanFilter>::iterator iter=MKF.begin();iter!=MKF.end();)//遍历历史卡曼滤波对象
	{
		Mat prediction = iter->second.predict(); //得到历史卡曼对象的预测结果
		 
		My_Person person;
		person.P.x=prediction.at<float>(0);
		person.P.y=prediction.at<float>(1);
		person.index=iter->first;

		if( fabs(person.P.x-pose.x) >r || fabs(person.P.y-pose.y) >r )	//超出范围，认为该障碍物已经走出视野，将该卡曼滤波对象删除
		{
			MKF.erase(iter++);
		}	
		else
		{
			predict_pose.push_back(person);	//保存预测结果，以进行输出
			++iter;
		}
	
	}
}

/*
* 根据匈牙利匹配结果(measure_pose)，更新卡曼滤波器对象map(MKF)：
* 如果匹配对象原本存在--更新卡曼滤波器对象；（可能障碍物本就存在视野中）
* 如果匹配对象原本不存在--创建新的卡曼滤波对象；（可能障碍物初次来到视野中）
* 如果原本存在的卡曼滤波器对象没有得到更新--删除卡曼滤波器对象；（可能障碍物从视野中消失）
*/
void UpdataPose(vector<My_Person>&measure_pose)
{
	for(map<int,KalmanFilter>::iterator iter=MKF.begin();iter!=MKF.end();)
	{
		bool Is_find=false;
		for(int i=0;i<measure_pose.size();i++)
		{
			if(measure_pose[i].index==iter->first)	//1、更新卡曼滤波器对象
			{
				Is_find=true;
				Mat measurement = Mat::zeros(measureNum, 1, CV_32F); 
				measurement.at<float>(0) = (float)measure_pose[i].P.x;  
				measurement.at<float>(1) = (float)measure_pose[i].P.y;
				MKF[measure_pose[i].index].correct(measurement); 

				break;
			}
		}

		if(Is_find==false)
			MKF.erase(iter++);						//2、删除没得到更新的卡曼滤波器对象
		else
			++iter;	
	}

	for(int i=0;i<measure_pose.size();i++)
	{
		if(!MKF.count(measure_pose[i].index))		//3、创建新的卡曼滤波对象
		{
			MKF[measure_pose[i].index].init(stateNum, measureNum, 0); 
	 		MKF[measure_pose[i].index].transitionMatrix = *(Mat_<float>(4, 4) <<1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,1);  //转移矩阵A  
			setIdentity(MKF[measure_pose[i].index].measurementMatrix);                                             //测量矩阵H  
			setIdentity(MKF[measure_pose[i].index].processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q  
			setIdentity(MKF[measure_pose[i].index].measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R  
			setIdentity(MKF[measure_pose[i].index].errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵

			//根据第一次检测结果设置初始位姿
			MKF[measure_pose[i].index].statePost.at<float>(0)=(float)measure_pose[i].P.x;
			MKF[measure_pose[i].index].statePost.at<float>(1)=(float)measure_pose[i].P.y;
		}
	}

}


