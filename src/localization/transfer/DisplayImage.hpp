///=========include==========
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <ros/package.h>
#include <iostream>
///=========define==========
#define deg2rad 3.1415926/180
#define rad2deg 180/3.1415926
///=========include & define end==========
class Img{
private:
    ros::NodeHandle *nh;
    ros::Publisher MAP_PUB;
    std::vector<int8_t> ary;
public:
    Img(int argc, char** argv,const char* node_name);
    ~Img() {}
    void ros_comms_init();
//    topic publish or subscriber
    void map_pub();
    void load_map();
//    param
    ros::NodeHandle* getNodeHandle(){return nh;}
};
