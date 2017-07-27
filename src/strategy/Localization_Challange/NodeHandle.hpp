/**
 * @file NodeHandle.hpp
 *
 * @brief Ros communication central!
 *
 * @date July 2017
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef _NODEHANDLE_HPP_
#define _NODEHANDLE_HPP_
/*****************************************************************************
** Includes
*****************************************************************************/
#include "../common/BaseNode.h"
#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"
#include "Env.hpp"
#include "vision/Object.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "imu_3d/inertia.h"
/*****************************************************************************
** Define
*****************************************************************************/

class NodeHandle : public BaseNode
{
  public:
    ///         public member           ///
    ///         constructor             ///
    NodeHandle(int argc, char **argv);
    virtual ~NodeHandle() {}
    void setEnv(Environment *);
    void setParam(Parameter *);
    void setLocationPoint(LocationStruct *);
    void pubSpeed(Environment *);
    void getParameter();

  protected:
    ///         protected member        ///
    void ros_comms_init();

  private:
    ///         private member          ///
    ros::NodeHandle *node;
    ///         subscriber              ///
    ros::Subscriber GAMESTATE;
    ros::Subscriber SAVEPARAM;
    ros::Subscriber VISION;
    ros::Subscriber LOCATIONPOINT;
    ros::Subscriber ROBOTPOSE;
    ros::Subscriber IMU;
    ros::Publisher SPEED;
    Environment *_Env;
    LocationStruct *_Location;
    Parameter *_Param;
    // std::vector<double> SPlanning_Velocity;
    ///         private function        ///
    void subGameState(const std_msgs::Int32::ConstPtr &msg) { _Env->GameState = msg->data; }
    void subSaveParam(const std_msgs::Int32::ConstPtr &msg) { _Env->SaveParam = msg->data; }
    void subVision(const vision::Object::ConstPtr &);
    void subRobotPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);
    void subLocationPoint(const std_msgs::Float32MultiArray::ConstPtr &);
    void subIMU(const imu_3d::inertia::ConstPtr &);
    void Transfer(Environment *);
    void VelocityPlanning(Environment *);
};
#endif