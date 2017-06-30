#include <ros/ros.h>
#include "monitor.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "monitor");//initial
  InterfaceProc ip;//class:InterfaceProc(interface.hpp)
  ros::spin();//listen topic value
  return 0;
}
