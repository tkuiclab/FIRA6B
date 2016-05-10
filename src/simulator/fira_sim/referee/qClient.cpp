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

QClient::QClient(int argc, char** argv ) :
    QNode(argc,argv,"QClient_client")
    {}

void QClient::ros_comms_init() {
    ros::NodeHandle n;
    send_client = n.serviceClient<fira_sim::WorldPluginSrv>(SrvName);
}

void QClient::sendStr(std::string A){
    //qMSN::TwoInts srv;
    //msn_srv_client::MSN srv;
    fira_sim::WorldPluginSrv srv;

    srv.request.command = A;
    if (send_client.call(srv))
    {
        Blue_G = srv.response.blueGoal;
        Yellow_G = srv.response.yellowGoal;
        Game_S = srv.response.gameState;
        Who_B = srv.response.whosBall;
        State_History = srv.response.stateHistory;

//        ROS_INFO("blue: %ld", (long int)srv.response.blueGoal);
//        ROS_INFO("Yellow: %ld", (long int)srv.response.yellowGoal);
    }
    else
    {
      ROS_ERROR("Failed to call service %s",SrvName);
      return ;
    }
    return ;

}

void QClient::run() {


}

