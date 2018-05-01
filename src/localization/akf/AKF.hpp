/**
 * @file AKF.hpp
 *
 * @brief Adaptive Kalman Filter 
 *
 * @Date August 2017
 *
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef _AKF_HPP_
#define _AKF_HPP_
/*****************************************************************************
** Incldue
*****************************************************************************/
#include <ros/ros.h>
#include "Env.hpp"
#include <math.h>

class AKF{
public:
    AKF(int argc, char** argv);
    ~AKF(){};
    void AKF_function(pose, pose);
    pose getAKF_pose();
private:
    void _InitParam();
    Kalman _kalman;
};

#endif