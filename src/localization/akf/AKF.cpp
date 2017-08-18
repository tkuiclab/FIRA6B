/**
 * @file AKF.cpp
 *
 * @brief Adaptive Kalman Filter 
 *
 * @Date August 2017
 *
 **/
 
/*******************************************************
** @Include
*******************************************************/
#include "AKF.hpp"
/*******************************************************
** @Public
*******************************************************/
AKF::AKF(int argc,char** argv){
    printf("Adaptive Kalman Filter enable\n");
    _InitParam();
}
void AKF::AKF_function(pose amcl_pose, pose ekf_pose){
    _kalman.mea[0][0] = ekf_pose.x;           
    // _kalman.mea[0][1] = ekf_pose.y;   
    _kalman.mea[0][1] = ekf_pose.y-0.30;   // for localization     
    _kalman.mea[1][0] = amcl_pose.x;              
    _kalman.mea[1][1] = amcl_pose.y;            
    for(int i=0; i<2; i++)
        for(int j=0; j<2; j++){
            _kalman.fp[i][j] = fabs(_kalman.est[j] - _kalman.mea_p[i][j]);
            _kalman.mea_p[i][j] = _kalman.mea[i][j];
            _kalman.kg[i][j] = (_kalman.a[i]*_kalman.kg[i][j]+_kalman.fp[i][j])/
            (_kalman.a[i]*(1+_kalman.kg[i][j])+_kalman.fp[i][j]);
            _kalman.est[j] = _kalman.est[j]+(_kalman.a[i]*_kalman.kg[i][j]+_kalman.fp[i][j])*
            (_kalman.mea[i][j]-_kalman.est[j])/(_kalman.a[i]*(1+_kalman.kg[i][j])+_kalman.fp[i][j]);
        }
    printf("amcl.x=%dcm\tamcl_y=%dcm\n",(int)(_kalman.mea[1][0]*100),(int)(_kalman.mea[1][1]*100));
    printf("ekf.x=%dcm\tekf_y=%dcm\n",(int)(_kalman.mea[0][0]*100),(int)(_kalman.mea[0][1]*100));
    printf("akf.x=%dcm\takf_y=%dcm\n",(int)(_kalman.est[0]*100),(int)(_kalman.est[1]*100));
    printf("==============================================\n");               
}
pose AKF::getAKF_pose(){
    pose msg;
    msg.x = _kalman.est[0];
    msg.y = _kalman.est[1];
    return msg;
}
/*******************************************************
** @Private
*******************************************************/
void AKF::_InitParam(){
    printf("Param initialize!!\n");
    double kp = 7;                        // kalman parameter
    _kalman.w[0] = 1 / pow(4.0,kp);       // kalman parameter w for ekf    value ∝ trust
    _kalman.w[1] = 1 / pow(2.0,kp);       // kalman parameter w for amcl    value ∝ trust
    _kalman.a[0] = pow(4.0,kp);           // kalman parameter a for ekf    value ∝ 1/trust
    _kalman.a[1] = pow(2.0,kp);           // kalman parameter a for amcl    value ∝ 1/trust
    for(int i=0; i<2; i++)
        for(int j=0; j<2; j++){
            _kalman.mea_p[i][j] = 0;               // last measure data
            _kalman.mea[i][j] = 0;                 // measure data
            _kalman.kg[i][j] = 0;                  // kalman gain
            _kalman.est[j] = 0;                 // estimate value
            _kalman.fp[i][j] = 0;                  //filter performance parameter
        }
}
