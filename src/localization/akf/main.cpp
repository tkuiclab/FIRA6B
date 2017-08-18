#include <stdio.h>
#include <stdlib.h>
#include "AKF.hpp"
#include "NodeHandle.hpp"
int main(int argc, char **argv){
    ros::init(argc, argv, "AKF_node");
    NodeHandle nodehandle(argc, argv);
    AKF kalmanfilter(argc, argv);
    pose amcl,ekf,akf;
    ros::Rate loop_rate(2000);
    while(ros::ok()){
        amcl = nodehandle.GetAmclPose();
        ekf = nodehandle.GetEkfPose();
        kalmanfilter.AKF_function(amcl,ekf);
        akf = kalmanfilter.getAKF_pose();
        // printf("x=%lf\ty=%lf\n",akf.x,akf.y);
        nodehandle.PubAkfPose(akf);
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
