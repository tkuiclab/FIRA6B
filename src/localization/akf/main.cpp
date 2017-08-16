#include <stdio.h>
#include <stdlib.h>
#include "AKF.hpp"
#include "NodeHandle.hpp"
int main(int argc, char **argv){
    ros::init(argc, argv, "AKF_node");
    NodeHandle nodehandle(argc, argv);
    AKF kalmanfilter(argc, argv);
    pose amcl,ekf;
    ros::Rate loop_rate(100);
    while(ros::ok()){
        amcl = nodehandle.get_amcl_pose();
        ekf = nodehandle.get_ekf_pose();
        kalmanfilter.AKF_function(amcl,ekf);
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}