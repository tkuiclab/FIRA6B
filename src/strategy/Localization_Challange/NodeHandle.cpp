/**
 * @file NodeHandle.hpp
 *
 * @brief Ros communication central!
 *
 * @date July 2017
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include "NodeHandle.hpp"

NodeHandle::NodeHandle(int argc, char **argv) : BaseNode(argc, argv, NODE_NAME)
{
    _Env = new Environment;
    _Location = new LocationStruct;
    // _Param->NodeHandle.SPlanning_Velocity.resize(10);
    _Env->GameState = 0;
    _Env->SaveParam = 0;
}
void NodeHandle::ros_comms_init()
{
    node = new ros::NodeHandle();
    GAMESTATE = node->subscribe<std_msgs::Int32>(GAME_STATE_TOPIC, 1000, &NodeHandle::subGameState, this);
    SAVEPARAM = node->subscribe<std_msgs::Int32>(SAVE_PARAM_TOPIC, 1000, &NodeHandle::subSaveParam, this);
    VISION = node->subscribe<vision::Object>(VISION_TOPIC, 1, &NodeHandle::subVision, this);
    SPEED = node->advertise<geometry_msgs::Twist>(SPEED_TOPIC, 1000);
    ROBOTPOSE = node->subscribe<geometry_msgs::PoseWithCovarianceStamped>(ROBOTPOSE_TOPIC, 1000, &NodeHandle::subRobotPose, this);
    LOCATIONPOINT = node->subscribe<std_msgs::Float32MultiArray>(LOCATIONPOINT_TOPIC, 1000, &NodeHandle::subLocationPoint, this);
    IMU = node->subscribe<imu_3d::inertia>("/imu_3d", 1, &NodeHandle::subIMU, this);
}
void NodeHandle::setEnv(Environment *Env)
{
    _Env = Env;
}
void NodeHandle::setParam(Parameter *Param)
{
    _Param = Param;
}
void NodeHandle::setLocationPoint(LocationStruct *LocationPoint)
{
    _Location = LocationPoint;
}
void NodeHandle::subVision(const vision::Object::ConstPtr &msg)
{
    double ball_distance;
    _Env->Robot.ball.distance = msg->ball_dis / 100.0;
    _Env->Robot.ball.angle = msg->ball_ang;
}
void NodeHandle::subRobotPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    _Env->Robot.pos.x = msg->pose.pose.position.x;
    _Env->Robot.pos.y = msg->pose.pose.position.y;
}
void NodeHandle::subLocationPoint(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    double _radius = 0.2;
    for (int i = 0; i < 5; i++)
    {
        _Location->LocationPoint[i].y = -msg->data[i * 2] / 100;
        _Location->LocationPoint[i].x = msg->data[i * 2 + 1] / 100;
        _Location->LocationPoint[i].angle = atan2(_Location->LocationPoint[i].y, _Location->LocationPoint[i].x) * RAD2DEG;
    }
    for (int i = 0; i < 5; i++)
    {
        if (i < 4)
            _Location->MiddlePoint[i].angle = atan2(_Location->LocationPoint[i].y + _Location->LocationPoint[i + 1].y, _Location->LocationPoint[i].x + _Location->LocationPoint[i + 1].x) * RAD2DEG;
        else
            _Location->MiddlePoint[i].angle = _Location->LocationPoint[i].angle;
        _Location->MiddlePoint[i].x = 0.5 * cos(_Location->MiddlePoint[i].angle * DEG2RAD);
        _Location->MiddlePoint[i].y = 0.5 * sin(_Location->MiddlePoint[i].angle * DEG2RAD);
    }
}
void NodeHandle::subIMU(const imu_3d::inertia::ConstPtr &msg)
{
    _Env->Robot.pos.angle = (((msg->yaw - M_PI) * RAD2DEG) > 0) ? -((msg->yaw - M_PI) * RAD2DEG - 180) : -((msg->yaw - M_PI) * RAD2DEG + 180);
}
void NodeHandle::pubSpeed(Environment *Env)
{
    Transfer(Env);
    // VelocityPlanning(Env);
    geometry_msgs::Twist SpeedMsg;
    SpeedMsg.linear.x = Env->Robot.v_x;
    SpeedMsg.linear.y = Env->Robot.v_y;
    SpeedMsg.angular.z = Env->Robot.v_yaw;
    SPEED.publish(SpeedMsg);
}
void NodeHandle::Transfer(Environment *Env)
{
    double Distance = hypot(Env->Robot.v_x, Env->Robot.v_y);
    double alpha = atan2(Env->Robot.v_y, Env->Robot.v_x) * RAD2DEG;
    double angle = Env->Robot.v_yaw;
    bool IsVectorZero = 0;
    double DistanceMax = _Param->NodeHandle.SPlanning_Velocity[0];
    double DistanceMin = _Param->NodeHandle.SPlanning_Velocity[1];
    double VelocityMax = _Param->NodeHandle.SPlanning_Velocity[2];
    double VelocityMin = _Param->NodeHandle.SPlanning_Velocity[3];
    double AngularVelocityMax = _Param->NodeHandle.SPlanning_Velocity[4];
    double AngularVelocityMin = _Param->NodeHandle.SPlanning_Velocity[5];
    double AngleMax = _Param->NodeHandle.SPlanning_Velocity[6];
    double AngleMin = _Param->NodeHandle.SPlanning_Velocity[7];
    double VelocityLength;
    double AngularVelocity;
    if (Distance == 0)
        IsVectorZero = 1;
    else if (Distance > DistanceMax)
        VelocityLength = VelocityMax;
    else if (Distance < DistanceMin)
        VelocityLength = VelocityMin;
    else
        VelocityLength = (VelocityMax - VelocityMin) * (cos(pi * ((Distance - DistanceMin) / (DistanceMax - DistanceMin) - 1)) + 1) / 2 + VelocityMin;
    if (fabs(angle) < 0.1)
        AngularVelocity = 0;
    else if (fabs(angle) > AngleMax)
        AngularVelocity = AngularVelocityMax;
    else if (fabs(angle) < AngleMin)
        AngularVelocity = AngularVelocityMin;
    else
        AngularVelocity = (AngularVelocityMax - AngularVelocityMin) * (cos(pi * ((fabs(angle) - AngleMin) / (AngleMax - AngleMin) - 1)) + 1) / 2 + AngularVelocityMin;
    if (angle < 0)
        AngularVelocity = -AngularVelocity;
    if (IsVectorZero)
    {
        Env->Robot.v_x = 0;
        Env->Robot.v_y = 0;
    }
    else
    {
        Env->Robot.v_x = VelocityLength * cos(alpha * DEG2RAD);
        Env->Robot.v_y = VelocityLength * sin(alpha * DEG2RAD);
    }
    Env->Robot.v_yaw = AngularVelocity;
}
void NodeHandle::VelocityPlanning(Environment *Env)
{
    //
}
void NodeHandle::getParameter()
{
    printf("success getting parameter!!\n");
    node->getParam("/FIRA/SPlanning_Velocity", _Param->NodeHandle.SPlanning_Velocity);
    node->getParam("/FIRA/hold_condition", _Param->Strategy.HoldBall_Condition);
}
