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

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include "../common/qnode.hpp"
#endif
#include <string>
#include <std_msgs/String.h>
#include <QFuture>
#include <qtconcurrentrun.h>
#include "FIRA_Sim/status.h"
#include "imu_3d/inertia.h"
#include "vision/Info.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "stdio.h"
#include <eigen3/Eigen/Dense>

#include "../../AllPrjCommon/ROSNameDef.h"
/*****************************************************************************
** Class
*****************************************************************************/

using namespace Eigen;

typedef struct{
    double shift_x;
    double shift_y;
    double rotation;
}imu;

class QClient : public QNode {

public:
    QClient(int argc, char** argv);
    virtual ~QClient() {}
    void run();
    void ros_comms_init();
    void reflash();
    void reset();

    int white_lineDist[90];
    imu return_accel();
    int *return_whiteline();
    Vector2i y_goal;
    Vector2i return_doorDist();



private:
    void imu_data(const imu_3d::inertia::ConstPtr& data);
    void whiteline_data(const std_msgs::Int32MultiArray::ConstPtr& array);
    void get_info(const vision::Info::ConstPtr& info);

    ros::Subscriber imu_sub;
    ros::Subscriber whiteline_sub;
    ros::Subscriber doorDist_sub;


    ros::NodeHandle *n;
};

#endif /* QClient_NODE_HPP_ */
