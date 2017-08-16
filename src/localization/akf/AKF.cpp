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
    InitParam();
}
void AKF::AKF_function(pose amcl_pose, pose ekf_pose){
    _kalman.mea[0][0] = amcl_pose.x;           
    _kalman.mea[0][1] = amcl_pose.y;          
    _kalman.mea[1][0] = ekf_pose.x;              
    _kalman.mea[1][1] = ekf_pose.y;                
    for(int i=0; i<2; i++)
        for(int j=0; j<2; j++)
            _kalman.fp[i][j] = fabs(_kalman.est[j] - _kalman.mea_p[i][j]);   
    _kalman.mea_p[0][0] = _kalman.mea[0][0];             
    _kalman.mea_p[0][1] = _kalman.mea[0][1];             
    _kalman.mea_p[1][0] = _kalman.mea[1][0];                
    _kalman.mea_p[1][1] = _kalman.mea[1][1];
    for(int i=0; i<2; i++)
        for(int j=0; j<2; j++){
            _kalman.kg[i][j] = (_kalman.a[i]*_kalman.kg[i][j]+_kalman.fp[i][j])/
            (_kalman.a[i]*(1+_kalman.kg[i][j])+_kalman.fp[i][j]);
            _kalman.est[j] = _kalman.est[j]+(_kalman.a[i]*_kalman.kg[i][j]+_kalman.fp[i][j])*
            (_kalman.mea[i][j]-_kalman.est[j])/(_kalman.a[i]*(1+_kalman.kg[i][j])+_kalman.fp[i][j]);
        }
    printf("_kalman.mea[0][0]=%lf\t_kalman.est[0]=%lf\n",_kalman.mea[0][0],_kalman.est[0]);               
}
void AKF::InitParam(){
    printf("Param initialize!!\n");
    double kp = 8;      // kalman parameter
    _kalman.w[0] = 1 / pow(2.0,kp);       // kalman parameter w
    _kalman.w[1] = 1 / pow(5.0,kp);       // kalman parameter w
    _kalman.a[0] = 1 / pow(2.0,-kp);      // kalman parameter a
    _kalman.a[1] = 1 / pow(2.0,-kp);      // kalman parameter a
    for(int i=0; i<2; i++)
        for(int j=0; j<2; j++){
            _kalman.mea_p[i][j] = 0;               // last measure data
            _kalman.mea[i][j] = 0;                 // measure data
            _kalman.kg[i][j] = 0;                  // kalman gain
            _kalman.est[j] = 0;                 // estimate value
            _kalman.fp[i][j] = 0;                  //filter performance parameter
        }
}