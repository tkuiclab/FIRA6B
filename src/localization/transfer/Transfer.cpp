#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include "NodeHandle.hpp"
int main(int argc, char **argv){
    Client mNodeHandle(argc,argv,"NodeHandle");
    mNodeHandle.ros_comms_init();
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        mNodeHandle.loadParam(mNodeHandle.getNodeHandle());
        mNodeHandle.whiteline_pub();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

