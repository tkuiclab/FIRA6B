#include <ros/ros.h>
#include "white_and_black.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "whiteLine");
  while(ros::ok()){
  InterfaceProc imageCb;
  imageCb.Parameter_getting(1);
  ros::spinOnce();
  }
  return 0;
}
