#include <ros/ros.h>
#include "monitor_goalkeeper.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "monitor");//initial
  InterfaceProc ip;//class:InterfaceProc(interface.hpp)
  //ip.Parameter_getting(1);
  ros::spin();//listen topic value

 return 0;
}
