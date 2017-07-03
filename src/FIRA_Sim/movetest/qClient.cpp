/**
 * @file /QClient_server/QClient.cpp
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
#include "qClient.hpp"
#include <std_msgs/String.h>



/*****************************************************************************
** Implementation
*****************************************************************************/
imu accel;
double pi = 3.14159;
double destination_x;
double destination_y;

void QClient::get_info(const vision::Info::ConstPtr& info)
{
    y_goal(0) = info->y_x;
    y_goal(1) = info->y_y;
}

void QClient::whiteline_data(const std_msgs::Int32MultiArray::ConstPtr& array)
{
    int i = 0;
    for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        white_lineDist[i] = *it;
        i++;
    }
}

void QClient::imu_data(const imu_3d::inertia::ConstPtr& data)
{
    accel.shift_x = data->shift_x;
    accel.shift_y = data->shift_y;
    accel.rotation = data->yaw;
}

QClient::QClient(int argc, char** argv) :
    QNode(argc,argv,"QClient_client")
    {
        ros::init(init_argc, init_argv, "PF_connect");
    }
void QClient::ros_comms_init()
{
    n = new ros::NodeHandle();

    imu_sub = n->subscribe("/imu_3d",1000,&QClient::imu_data,this);
    whiteline_sub = n->subscribe("/WhitlRealDis",1000,&QClient::whiteline_data,this);
    doorDist_sub = n->subscribe(Vision_Segmentation_Topic,1000,&QClient::get_info,this);
}
//更新場地資訊
void QClient::reflash(){

}

void QClient::run() {
    ros::Rate loop_rate(1000);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
imu QClient::return_accel()
{
    return accel;
}
int *QClient::return_whiteline()
{
    return white_lineDist;
}
Vector2i QClient::return_doorDist()
{
    return y_goal;
}
