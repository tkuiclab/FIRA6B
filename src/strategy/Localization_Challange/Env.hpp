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
/*****************************************************************************
** Define
*****************************************************************************/
#define NODE_NAME "LocalizationChallange"
#define GAME_STATE_TOPIC "/FIRA/GameState"
#define SAVE_PARAM_TOPIC "/FIRA/SaveParam"
#define VISION_TOPIC "/vision/object"
#define SPEED_TOPIC "/motion/cmd_vel"
#define ROBOTPOSE_TOPIC "/amcl_pose"
#define LOCATIONPOINT_TOPIC "/FIRA/Location"
#define pi M_PI
#define RAD2DEG 180/pi
#define DEG2RAD pi/180
#define FAULTEXECUTING 0
#define SUCCESSEXECUTING 1 
#define TRUE 1
#define FALSE 0

typedef struct{
    double x,y;
    double angle,distance;
}VisionData;
typedef struct{
    VisionData pos;
    VisionData ball;
    VisionData goal;
    VisionData op_goal;
    double v_x,v_y,v_yaw;
}RobotData;
typedef struct{
    RobotData Robot;
    int GameState;
    int SaveParam;
}Environment;
typedef struct{
    double x,y;
    double angle;
}Point;
typedef struct{
    Point LocationPoint[5];
    Point MiddlePoint[5];
}LocationStruct;
#endif