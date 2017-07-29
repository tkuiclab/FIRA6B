/**
 * @file Strategy.hpp
 *
 * @brief Localization challange localization class
 *
 * @date July 2017
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef _ENV_HPP_
#define _ENV_HPP_
/*****************************************************************************
** Includes
*****************************************************************************/
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include "imu_3d/inertia.h"
#include <string>
/*****************************************************************************
** Define
*****************************************************************************/
// Topic name
#define NODE_NAME "LocalizationChallange"
#define GAME_STATE_TOPIC "/FIRA/GameState"
#define SAVE_PARAM_TOPIC "/FIRA/SaveParam"
#define VISION_TOPIC "/vision/object"
#define SPEED_TOPIC "/motion/cmd_vel"
#define ROBOTPOSE_TOPIC "/amcl_pose"
#define LOCATIONPOINT_TOPIC "/FIRA/Location"
// Strategy.cpp
#define STATE_HALT 0
#define STATE_LOCALIZATION 9
// General parameter
#define pi M_PI
#define RAD2DEG 180 / pi
#define DEG2RAD pi / 180
#define FAULTEXECUTING 0
#define SUCCESSEXECUTING 1
#define TRUE 1
#define FALSE 0

typedef struct
{
    double x, y;
    double angle, distance;
} VisionData;
typedef struct
{
    VisionData pos;
    VisionData ball;
    VisionData goal;
    VisionData op_goal;
    double v_x, v_y, v_yaw;
} RobotData;
typedef struct
{
    RobotData Robot;
    int GameState;
    int SaveParam;
} Environment;
typedef struct
{
    double x, y;
    double angle;
} Point;
typedef struct
{
    Point LocationPoint[5];
    Point MiddlePoint[5];
} LocationStruct;
typedef struct
{
    double x, y, yaw;
} Vector3D;
typedef struct
{
    std::vector<double> HoldBall_Condition;
} Strategy_Parameter;
typedef struct
{
    std::vector<double> SPlanning_Velocity;
} NodeHandle_Parameter;
typedef struct
{
    Strategy_Parameter Strategy;
    NodeHandle_Parameter NodeHandle;
} Parameter;
typedef struct
{
    Point TargetPoint[10];
    int size;
} TargetStruct;
#endif