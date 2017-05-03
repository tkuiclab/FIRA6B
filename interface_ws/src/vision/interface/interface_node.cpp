#include <ros/ros.h>
#include "interface.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interface_node");//initial
  InterfaceProc ip;//class:InterfaceProc(interface.hpp)
  ros::spin();//listen topic value
  return 0;
}
