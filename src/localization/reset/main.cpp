/* @ File main.cpp
 *
 * @ Brief control the reset command to amcl initial pose
 * 
 * @ Date Octorber 2017
 */
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include "NodeHandle.hpp"
int main(int argc, char **argv){
    
    ros::init(argc, argv, "reset_controller");
    Client mNodeHandle(argc,argv,"NodeHandle");
    mNodeHandle.ros_comms_init();
    sleep(0.5);
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        mNodeHandle.loadParam(mNodeHandle.getNodeHandle());
        ros::spinOnce();
        printf("%d\n",mNodeHandle.GetResetCommand());
        loop_rate.sleep();
    }
}