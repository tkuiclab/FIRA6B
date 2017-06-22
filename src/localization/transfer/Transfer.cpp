#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include "NodeHandle.hpp"
#include "DisplayImage.hpp"
int main(int argc, char **argv){
    
    ros::init(argc, argv, "localization_node");
    Client mNodeHandle(argc,argv,"NodeHandle");
    Img imgNode(argc,argv,"imgNodeHandle");
    mNodeHandle.ros_comms_init();
    imgNode.ros_comms_init();
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        imgNode.load_map();
        imgNode.map_pub();
        mNodeHandle.loadParam(mNodeHandle.getNodeHandle());
        mNodeHandle.whiteline_pub();
        mNodeHandle.odom_tf_pub();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

