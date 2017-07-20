/**
 * @file Strategy.cpp
 *
 * @brief Localization challange pathplan strategy
 *
 * @date July 2017
 **/
 /*****************************************************************************
** Includes
*****************************************************************************/
#include "Strategy.hpp"
Strategy::Strategy(){
    _LocationState = 0;
    _CurrentTarget = 0;
    _Location = new LocationStruct;
    _Env = new Environment;
}
void Strategy::GameState(int int_GameState){
    switch(int_GameState){
        case STATE_HALT:
            StrategyHalt();
            break;
        case STATE_LOCALIZATION:
            StrategyLocalization();
            break;
    }
}
void Strategy::StrategyHalt(){
    _Env->Robot.v_x = 0;
    _Env->Robot.v_y = 0;
    _Env->Robot.v_yaw = 0;
}
void Strategy::StrategyLocalization(){
    RobotData Robot;
    Robot.pos.x = _Env->Robot.pos.x;
    Robot.pos.y = _Env->Robot.pos.y;
    double imu = _Env->Robot.pos.angle;
    static int flag = TRUE;
    static double Begin_time = ros::Time::now().toSec();// init timer begin
    double Current_time = ros::Time::now().toSec();   // init timer end
    double v_x,v_y,v_yaw;
    double accelerate = 1;
    double slow_factor =1;
    static int IMU_state = 0;
    if(flag)
        Begin_time = Current_time;
    if(Current_time-Begin_time < accelerate)
        slow_factor = exp(-2.1+2*((Current_time-Begin_time)/accelerate));
    else
        slow_factor = 1;      
    switch(_LocationState){
        case forward:                 // Move to target poitn
            flag = FALSE;
            v_x = _Location->LocationPoint[_CurrentTarget].x - Robot.pos.x;
            v_y = _Location->LocationPoint[_CurrentTarget].y - Robot.pos.y;
            if(fabs(v_x)<=0.05 && fabs(v_y)<=0.05){
                _LocationState = back;
                flag = TRUE;
            }
            // printf("state:forward\tSchedule=%d\n==========================\n",_CurrentTarget);
            break;
        case back:                 // Back to middle circle
            flag = FALSE;
            v_x = _Location->MiddlePoint[_CurrentTarget].x - Robot.pos.x;
            v_y = _Location->MiddlePoint[_CurrentTarget].y - Robot.pos.y;
            if(fabs(v_x)<=0.05 && fabs(v_y)<=0.05){
                if(_CurrentTarget==4)
                    _LocationState = finish;
                else{
                    _LocationState = forward;
                    flag = TRUE;
                }
            _CurrentTarget++;}
            // printf("state:back\tSchedule=%d\n==========================\n",_CurrentTarget);
            break;
        case finish:
            v_x = 0;
            v_y = 0;
            break;
        case error:
            printf("ERROR STATE\n");
            exit(FAULTEXECUTING);
            break;
        default:                // ERROR SIGNAL
            printf("ERROR STATE\n");
            exit(FAULTEXECUTING);
    }
    if(imu>5)
        IMU_state = 1;
    else if(imu<-5)
        IMU_state = 2;
    else if(fabs(imu)<3)
        IMU_state = 0;
    else
        //do nothing
    switch(IMU_state){
    case 0:
        v_yaw = 0;
        break;
    case 1:
        v_yaw = -1;
        break;
    case 2:
        v_yaw = 1;
        break;
    default:
        break;
    }
    _Env->Robot.v_x = v_x*slow_factor;
    _Env->Robot.v_y = v_y*slow_factor;
    _Env->Robot.v_yaw = v_yaw;
}