/**
 * @file /QServer_server/QServer.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <sstream>
#include "qInterface.hpp"
#include <std_msgs/String.h>

/*****************************************************************************
** Implementation
*****************************************************************************/
//#define QT_NO_KEYWORDS

QInterface::QInterface(int argc, char** argv ) :
    QNode(argc,argv,"QInterface")
    {}

void QInterface::imageCb(const sensor_msgs::ImageConstPtr& msg){
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}
void QInterface::object_data(const vision::Object & msg){
    image_fps = msg.fps;
    ball_LR = msg.ball_LR;
    ball_x = msg.ball_x;
    ball_y = msg.ball_y;
    ball_ang = msg.ball_ang;
    ball_dis = msg.ball_dis;

    blue_LR = msg.blue_LR;
    blue_x = msg.blue_x;
    blue_y = msg.blue_y;
    blue_ang = msg.blue_ang;
    blue_dis = msg.blue_dis;

    yellow_LR = msg.yellow_LR;
    yellow_x = msg.yellow_x;
    yellow_y = msg.yellow_y;
    yellow_ang = msg.yellow_ang;
    yellow_dis = msg.yellow_dis;
}

void QInterface::ros_comms_init() {
    nh  = new ros::NodeHandle();
    it_ = new image_transport::ImageTransport(*nh);
    sensor_msgs::ImageConstPtr msg;
    image_sub_ = it_->subscribe("/camera/image_raw" ,10 ,&QInterface::imageCb ,this);
    object_sub = nh->subscribe("/vision/object",10 ,&QInterface::object_data ,this);
}


void QInterface::run() {
    ros::spin();
    Q_EMIT rosShutdown();
}
