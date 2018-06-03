#include "ros/ros.h"
#include "math.h"
#include "sensor_msgs/Joy.h"
#include "Base_control/ControlCmd.h"
#include <iostream>


class Joyrecv
{
	public:
		ros::NodeHandle n;
		ros::Subscriber suber;
		ros::ServiceClient clienter;
		int V,W;

		void handleRecv(const sensor_msgs::Joy::ConstPtr& msg);
		Joyrecv();
		~Joyrecv();
};

Joyrecv::Joyrecv()
{
	suber = n.subscribe("joy", 1, &Joyrecv::handleRecv,this);

	clienter = n.serviceClient<Base_control::ControlCmd>("Controler");
	V = 0;
	W = 0;
}

Joyrecv::~Joyrecv()
{


}


bool anjian = false;
void Joyrecv::handleRecv(const sensor_msgs::Joy::ConstPtr& msg)
{

	std::cout<<"receive....................................."<<std::endl;

	if(msg->buttons[1]==1)
	{	
		std::cout<<"stop car................. "<<std::endl;
		
		V = 0;
		W = 0;
		anjian = true;

		std::cout<<"now V W is "<<std::endl;
		std::cout<<V<<" "<<W<<std::endl; 

		std::cout<<"msg->buttons[1] "<<msg->buttons[1]<<std::endl;

		std::cout<<" msg->axes[0] "<< msg->axes[0]<<" "<< msg->axes[1]<<std::endl;

		
		
		Base_control::ControlCmd srv;
		srv.request.xx = V;
     		srv.request.yy = W;

		if (clienter.call(srv))
     		{
      			 ROS_INFO("Set vel=%d,omiga=%d",srv.request.xx,srv.request.yy);
    		}
     		else
     		{
       			ROS_ERROR("Failed to call service add_two_ints");
     		}
	}
	else
	{
		if(!anjian)
		{
			float Y = msg->axes[0];
			float X = msg->axes[1];

			

			if((msg->axes[0]<0.01 && msg->axes[0]>-0.01))
			{	
				Y = 0;
			}
			
			if((msg->axes[1]<0.01 && msg->axes[1]>-0.01))
			{	
				X = 0;
			}

			float Vv;
			float Wv;
			float angle;

			if((msg->axes[0]<0.01 && msg->axes[0]>-0.01) && (msg->axes[1]<0.01 && msg->axes[1]>-0.01))
			{
				Vv = 0;
				Wv = 0;
				
			}
			else
			{
				//angle = atan2(Y,X);
		
				//Vv = 500 * sin(angle);
				//Wv = 500 * cos(angle);

				Vv = 500 * Y;
				Wv = 500 * X;


			}			

			/*float angle = atan2(Y,X);
		
			float Vv = 500 * sin(angle);
			float Wv = 500 * cos(angle);*/

			V = Wv;
			W = Vv;

			std::cout<<"now V W is "<<std::endl;
			std::cout<<V<<" "<<W<<std::endl; 

			std::cout<<"msg->buttons[1] "<<msg->buttons[1]<<std::endl;

			std::cout<<" msg->axes[0] "<< msg->axes[0]<<" "<< msg->axes[1]<<std::endl;

			//std::cout<<" angle "<<angle<<std::endl;

			Base_control::ControlCmd srv;
			srv.request.xx = V;
     			srv.request.yy = W;

			if(clienter.call(srv))
     			{
      				 ROS_INFO("Set vel=%d,omiga=%d",srv.request.xx,srv.request.yy);
    			}
     			else
     			{
       				ROS_ERROR("Failed to call service add_two_ints");
     			}
		}
		else
		{
			anjian = false;
		}
		
	}
}

int main(int argc , char ** argv)
{
	ros::init(argc,argv,"app_Joystick_controller");
		Joyrecv Joyrecver;
	ros::spin();
}
