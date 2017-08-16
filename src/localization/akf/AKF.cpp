/**
 * @file AKF.cpp
 *
 * @brief Adaptive Kalman Filter 
 *
 * @Date August 2017
 *
 **/
#include "AKF.hpp"
AKF::AKF(int argc,char** argv){
    printf("Adaptive Kalman Filter enable\n");
}
void AKF::AKF_function(pose amcl_pose, pose ekf_pose){
    InitParam();
}
void AKF::InitParam(){
    double kp = 8;      // kalman parameter
    _kalman.w[0] = 1/pow(2.0,kp);       // kalman parameter w
    _kalman.w[1] = 1/pow(2.0,kp);       // kalman parameter w
    _kalman.a[0] = 1/pow(2.0,-kp);      // kalman parameter a
    _kalman.a[1] = 1/pow(2.0,-kp);      // kalman parameter a
    _kalman.mea_p[0] = 0;               // last measure data
    _kalman.mea_p[1] = 0;               // last measure data
    _kalman.mea[0] = 0;                 // measure data
    _kalman.mea[1] = 0;                 // measure data
    _kalman.kg[0] = 0;                  // kalman gain
    _kalman.kg[1] = 0;                  // kalman gain
    _kalman.est[0] = 0;                 // estimate value
    _kalman.est[1] = 0;                 // estimate value
}