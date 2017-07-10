#include <ros/ros.h>
#include "monitor.hpp"

int main(int argc, char** argv)
{
ros::init(argc, argv, "monitor");//initial
 while(ros::ok()){
  
  InterfaceProc ip;//class:InterfaceProc(interface.hpp)
  ip.Parameter_getting(1);
  ros::spinOnce();//listen topic value
 }
 return 0;
}
