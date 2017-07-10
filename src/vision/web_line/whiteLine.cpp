#include <ros/ros.h>
#include "white_and_black.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "whiteLine");
  InterfaceProc imageCb;
  //imageCb.Parameter_getting(1);
  ros::spin();
  return 0;
}
