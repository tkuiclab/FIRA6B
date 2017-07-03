/**
 * @file /QClient_server/QClient.hpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef QClient_NODE_HPP_
#define QClient_NODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#define SrvName "/FIRA/Simulaotr/WorldPluginSrv"


#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include "../common/qnode.hpp"
//#include "msn_srv_client/MSN.h"
#include "fira_sim/WorldPluginSrv.h"

#endif

#include <string>
#include <std_msgs/String.h>

/*****************************************************************************
** Class
*****************************************************************************/

class QClient : public QNode {

public:
    QClient(int argc, char** argv);
    virtual ~QClient() {}
    void run();
    void ros_comms_init();

    void sendStr(std::string A);

    int Blue_G,Yellow_G,Game_S;
    std::string State_History,Who_B;

private:

    ros::ServiceClient send_client;
};

#endif /* QClient_NODE_HPP_ */
