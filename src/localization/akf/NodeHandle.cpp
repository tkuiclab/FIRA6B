/**
 * @file NodeHandle.cpp
 *
 * @brief Ros communication central!
 *
 * @Date August 2017
 *
 **/

/*******************************************************
** @Include
*******************************************************/
#include "NodeHandle.hpp"
/*******************************************************
** @Public
*******************************************************/
NodeHandle::NodeHandle(int argc, char **argv){
    printf("NodeHandle enable\n");
    _ros_comms_init();
    _InitParam();
}
pose NodeHandle::GetAmclPose(){
    return _amcl_pose;
}
pose NodeHandle::GetEkfPose(){
    return _ekf_pose;
}
void NodeHandle::PubAkfPose(pose akf){
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "AKF_pose";
    msg.pose.pose.position.x = akf.x;
    msg.pose.pose.position.y = akf.y;
    AKF_ROBOTPOSE.publish(msg);
}
/*******************************************************
** @Private
*******************************************************/
void NodeHandle::_InitParam(){
    _amcl_pose.x = 0;
    _amcl_pose.y = 0;
    _ekf_pose.x = 0;
    _ekf_pose.y = 0;
}
void NodeHandle::_ros_comms_init(){
    node = new ros::NodeHandle();
    AMCL_ROBOTPOSE = node->subscribe<geometry_msgs::PoseWithCovarianceStamped>(AMCL_ROBOTPOSE_TOPIC, 1000, &NodeHandle::_SubAmclRobotPose, this);
    EKF_ROBOTPOSE = node->subscribe<nav_msgs::Odometry>(EKF_ROBOTPOSE_TOPIC, 1000, &NodeHandle::_SubEkfRobotPose, this);
    AKF_ROBOTPOSE = node->advertise<geometry_msgs::PoseWithCovarianceStamped>(AKF_ROBOTPOSE_TOPIC, 1000);
}
void NodeHandle::_SubAmclRobotPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
    _amcl_pose.x = msg->pose.pose.position.x;
    _amcl_pose.y = msg->pose.pose.position.y;
}
void NodeHandle::_SubEkfRobotPose(const nav_msgs::Odometry::ConstPtr &msg){
    _ekf_pose.x = msg->pose.pose.position.x;
    _ekf_pose.y = msg->pose.pose.position.y;
}
