#include <ros/ros.h>
#include "image_converter.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "whiteLine");
  ImageConverter ic;
  while(ros::ok()){
    ic.get_center();
    ic.get_distance();
    ic.get_whitedata();
    ic.get_Camera();
    ros::spinOnce();
  }
  return 0;
}
