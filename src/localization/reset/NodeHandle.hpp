/* @File NodeHandle.hpp
 *
 * @Brief ros comms model
 * 
 * @Date Octorber 2017
 */
#ifndef _NODEHANDLE_HPP_
#define _NODEHANDLE_HPP_

///=========include==========
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "math.h"
#include "sensor_msgs/LaserScan.h"
#include "imu_3d/inertia.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <time.h>
///=========define==========
#define deg2rad 3.1415926/180
#define rad2deg 180/3.1415926
#define POSE_TOPIC "/vision/WhiteRealDis"
#define RESET_COMMAND "/motion/motionFB"
#define AMCL_INIT "/idon'tknow"
///=========include & define end==========
class Client{
private:
    ros::NodeHandle *nh;
    ros::Subscriber amcl_pose_sub;
    ros::Subscriber reset_command_sub;
    ros::Publisher reset_imu_pub;
    ros::Publisher amcl_init_pub;
    int __reset_command;
    struct position{
        float x,y;
    };
    position __reset_pose;
public:
    Client(int argc, char** argv,const char* node_name);
    ~Client(){}
    int GetResetCommand();
    void ros_comms_init();
//    topic publish or subscriber
    void AmclPoseSub(const geometry_msgs::Pose2D& msg);
    void ResetCommandSub(const std_msgs::Int32& msg);
    void ResetImuPub();
    void AmclInitPub();
//    param
    ros::NodeHandle* getNodeHandle(){return nh;}
};

#endif