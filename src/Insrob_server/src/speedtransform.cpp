#include "ros/ros.h"
#include "Insrob_server/Speed.h"
#include "Base_control/ControlCmd.h"

ros::ServiceClient client;
int lastlinear = 0;
int lasttwist = 0;

void Speed_Callback(const Insrob_server::Speed &speedData) {
	Base_control::ControlCmd srv;
	std::cout<<"server.linear "<<speedData.linear<<" server.twist "<<speedData.twist<<std::endl;

	if(speedData.linear == 0 && speedData.twist==0)
	{
		if(lastlinear>0)
		{
			for(int i=lastlinear;i>50;i=i-4)
			{
				srv.request.xx = i;
				srv.request.yy = 0;
				client.call(srv);
				usleep(15000);
			}
		}
		if(lastlinear<0)
		{
			for(int i=lastlinear;i<-50;i=i+4)
			{
				srv.request.xx = i;
				srv.request.yy = 0;
				client.call(srv);
				usleep(15000);
			}
		}
		lastlinear = 0.0;
		lasttwist = 0.0;
	}

	if(speedData.linear > 0)
	{
		if((lastlinear - speedData.linear)>0)
		{
			for(int i=lastlinear;i>speedData.linear;i=i-4)
			{
				srv.request.xx = i;
				srv.request.yy = speedData.twist;
				client.call(srv);
				usleep(20000);	
			}
		}

		if((lastlinear - speedData.linear)<0)
		{
			for(int i=lastlinear;i<speedData.linear;i=i+4)
			{
				srv.request.xx = i;
				srv.request.yy = speedData.twist;
				client.call(srv);
				usleep(20000);	
			}
		}
		lastlinear = speedData.linear;
		lasttwist = speedData.twist;
	}

	if(speedData.linear < 0)
	{
		if((lastlinear-speedData.linear)>0)
		{
			for(int i=lastlinear;i>speedData.linear;i=i-4)
			{
				srv.request.xx = i;
				srv.request.yy = speedData.twist;
				client.call(srv);
				usleep(20000);	
			}

		}

		if((lastlinear-speedData.linear)<0)
		{
			for(int i=lastlinear;i<speedData.linear;i=i+4)
			{
				srv.request.xx = i;
				srv.request.yy = speedData.twist;
				client.call(srv);
				usleep(20000);	
			}
		}

		lastlinear = speedData.linear;
		lasttwist = speedData.twist;
	}

	if(speedData.twist > 0)
	{
		if((lasttwist-speedData.twist)>0)
		{
			for(int i=lasttwist;i>speedData.twist;i=i-4)
			{
				srv.request.xx = speedData.linear;
				srv.request.yy = i;
				client.call(srv);
				usleep(20000);
			}

		}

		if((lastlinear-speedData.linear)<0)
		{
			for(int i=lasttwist;i<speedData.twist;i=i+4)
			{
				srv.request.xx = speedData.linear;
				srv.request.yy = i;
				client.call(srv);
				usleep(20000);	
			}
		}

		lastlinear = speedData.linear;
		lasttwist = speedData.twist;
	}

	if(speedData.twist < 0)
	{
		if((lastlinear-speedData.linear)>0)
		{
			for(int i=lasttwist;i>speedData.twist;i=i-4)
			{
				srv.request.xx = speedData.linear;
				srv.request.yy = i;
				client.call(srv);
				usleep(20000);	
			}

		}

		if((lastlinear-speedData.linear)<0)
		{
			for(int i=lasttwist;i<speedData.twist;i=i+4)
			{
				srv.request.xx = speedData.linear;
				srv.request.yy = i;
				client.call(srv);
				usleep(20000);	
			}
		}

		lastlinear = speedData.linear;
		lasttwist = speedData.twist;
	}

	srv.request.xx = speedData.linear;
	srv.request.yy = speedData.twist;
	client.call(srv);	
}

int main(int argc, char **argv){
	ros::init(argc, argv, "speed_transform");
	ros::NodeHandle n;
	client = n.serviceClient<Base_control::ControlCmd>("Controler");
	ros::Subscriber speed_sub = n.subscribe("/speed_control", 2, Speed_Callback);
	ROS_INFO("speed_transform start!!!");
	while(ros::ok()){
		ros::spinOnce();
	}
	ROS_INFO("over");	
	return 0;
}

