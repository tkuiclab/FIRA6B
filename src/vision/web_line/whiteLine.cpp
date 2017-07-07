#include <ros/ros.h>
#include "white_and_black.hpp"

int main(int argc, char** argv)
{
  while(ros::ok()){
  ros::init(argc, argv, "whiteLine");
  InterfaceProc imageCb;
  imageCb.Parameter_getting(1);
  ros::spinOnce();
  }
  return 0;
}
