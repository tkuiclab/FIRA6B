/**
 * @file NodeHandle.hpp
 *
 * @brief Ros communication central!
 *
 * @Date August 2017
 *
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef _NODEHANDLE_HPP_
#define _NODEHANDLE_HPP_
/*****************************************************************************
** Incldue
*****************************************************************************/
#include <ros/ros.h>
#include "Env.hpp"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class NodeHandle{
public:
NodeHandle(int argc,char** argv);
~NodeHandle(){};
pose GetAmclPose();
pose GetEkfPose();
void PubAkfPose(pose);
private:
pose _amcl_pose;
pose _ekf_pose;
pose _final_pose;
ros::NodeHandle *node;
ros::Subscriber EKF_ROBOTPOSE;
ros::Subscriber AMCL_ROBOTPOSE;
ros::Publisher AKF_ROBOTPOSE;
void _ros_comms_init();
void _InitParam();
void _SubAmclRobotPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);
void _SubEkfRobotPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);
};
#endif