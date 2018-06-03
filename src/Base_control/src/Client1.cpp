#include "ros/ros.h"
#include "Base_control/ControlCmd.h"
#include<iostream>
using namespace std;
#include <cstdlib>
    
 int main(int argc, char **argv)
 {
      int i;
      ros::init(argc, argv, "Client1");
     
     /*
      if (argc != 3)
     {
       ROS_INFO("usage: add_two_ints_client X Y");
       return 1;
     }
     */
   
     ros::NodeHandle n;
     ros::ServiceClient client = n.serviceClient<Base_control::ControlCmd>("Controler");
     Base_control::ControlCmd srv;
     
     //srv.request.a = atoll(argv[1]);
     //srv.request.b = atoll(argv[2]);
     ROS_INFO("Please Click in the vel and omiga:");
     
    //循环方式键入vel和omiga

     while(1)
    {
     cin>>srv.request.xx;
     cin>>srv.request.yy;
    //if(srv.request.xx==0&&srv.request.yy==0) break;
    if (client.call(srv))
     {
       ROS_INFO("Set vel=%d,omiga=%d",srv.request.xx,srv.request.yy);
     }
     else
     {
       ROS_ERROR("Failed to call service add_two_ints");
     }
     usleep(999000);// 1000--1ms 
    }

     
     return 0;
 }
