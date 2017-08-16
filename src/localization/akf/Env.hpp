/**
 * @file Env.hpp
 *
 * @brief Environment
 *
 * @Date August 2017
 *
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef _ENV_HPP_
#define _ENV_HPP_
#define AMCL_ROBOTPOSE_TOPIC "/amcl_pose"
#define EKF_ROBOTPOSE_TOPIC "/robot_pose_ekf/odom_combined"

typedef struct{
    double x;
    double y;
} pose;
typedef struct{
    double w[2];        // kalman parameter
    double a[2];        // kalman parameter
    double mea_p[2];    // last measure data
    double mea[2];      // measure data
    double kg[2];       // kalman gain
    double est[2];      // estimate value
} Kalman;
#endif