#include <ros/ros.h>
#include "image_converter.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "blackItem");
  ImageConverter ic;
  ros::spin();
  return 0;
}
