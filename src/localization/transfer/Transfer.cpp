#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include "NodeHandle.hpp"
#include "DisplayImage.hpp"
int main(int argc, char **argv){
    
    ros::init(argc, argv, "localization_node");
    Client mNodeHandle(argc,argv,"NodeHandle");
    Img imgNode(argc,argv,"imgNodeHandle");  //Building map
    mNodeHandle.ros_comms_init();
    imgNode.ros_comms_init();
    imgNode.load_map();             //Building map
    sleep(0.5);
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        mNodeHandle.loadParam(mNodeHandle.getNodeHandle());
        ros::spinOnce();
        // imgNode.map_pub();
        mNodeHandle.whiteline_pub();
        mNodeHandle.odom_tf_pub();
        // mNodeHandle.initialpose_pub();
        loop_rate.sleep();
    }
}