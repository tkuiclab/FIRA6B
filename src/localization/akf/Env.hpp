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
#define AKF_ROBOTPOSE_TOPIC "/akf_pose"
typedef struct{
    double x;
    double y;
} pose;
typedef struct{             // attention!![i][j]  i is num of sensors , j is axis of x or y  
    double w[2];            // kalman parameter w
    double a[2];            // kalman parameter a 
    double mea_p[2][2];     // last measure data   
    double mea[2][2];       // measure data
    double kg[2][2];        // kalman gain
    double est[2];          // estimate value
    double fp[2][2];        //filter performance parameter
} Kalman;
#endif