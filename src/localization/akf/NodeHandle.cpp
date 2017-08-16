/**
 * @file NodeHandle.cpp
 *
 * @brief Ros communication central!
 *
 * @Date August 2017
 *
 **/
#include "NodeHandle.hpp"
NodeHandle::NodeHandle(int argc, char **argv){
    printf("NodeHandle enable\n");
    ros_comms_init();
    InitParam();
}
void NodeHandle::ros_comms_init(){
    node = new ros::NodeHandle();
    AMCL_ROBOTPOSE = node->subscribe<geometry_msgs::PoseWithCovarianceStamped>(AMCL_ROBOTPOSE_TOPIC, 1000, &NodeHandle::subAmclRobotPose, this);
    EKF_ROBOTPOSE = node->subscribe<geometry_msgs::PoseWithCovarianceStamped>(EKF_ROBOTPOSE_TOPIC, 1000, &NodeHandle::subEkfRobotPose, this);
}
void NodeHandle::subAmclRobotPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
    _amcl_pose.x = msg->pose.pose.position.x;
    _amcl_pose.y = msg->pose.pose.position.y;
}
void NodeHandle::subEkfRobotPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
    _ekf_pose.x = msg->pose.pose.position.x;
    _ekf_pose.y = msg->pose.pose.position.y;
}
pose NodeHandle::get_amcl_pose(){
    return _amcl_pose;
}
pose NodeHandle::get_ekf_pose(){
    return _ekf_pose;
}
void NodeHandle::InitParam(){
    _amcl_pose.x = 0;
    _amcl_pose.y = 0;
    _ekf_pose.x = 0;
    _ekf_pose.y = 0;
}