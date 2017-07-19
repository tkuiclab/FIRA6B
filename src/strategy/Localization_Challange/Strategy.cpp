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
void Strategy::setEnv(Environment *Env){
    _Env = Env;
}
void Strategy::setLocationPoint(LocationStruct *LocationPoint){
    _Location = LocationPoint;
}
// Environment Strategy::getEnv(){
//     return *_Env;
// }
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
    double v_x,v_y;
    switch(_LocationState){
        case forward:                 // Move to target poitn
            v_x = _Location->LocationPoint[_CurrentTarget].x - Robot.pos.x;
            v_y = _Location->LocationPoint[_CurrentTarget].y - Robot.pos.y;
            if(fabs(v_x)<=0.1 && fabs(v_y)<=0.1)
                _LocationState = back;
            printf("x=%lf\ty=%lf\n",_Location->LocationPoint[_CurrentTarget].x,_Location->LocationPoint[_CurrentTarget].y);
            printf("VX=%f\tVY=%f\n",v_x,v_y);
            printf("state:forward\tSchedule=%d\n==========================\n",_CurrentTarget);
            break;
        case back:                 // Back to middle circle
            v_x = _Location->MiddlePoint[_CurrentTarget].x - Robot.pos.x;
            v_y = _Location->MiddlePoint[_CurrentTarget].y - Robot.pos.y;
            if(fabs(v_x)<=0.1 && fabs(v_y)<=0.1){
                if(_CurrentTarget==4)
                    _LocationState = finish;
                else
                    _LocationState = forward;
            _CurrentTarget++;}
            printf("x=%lf\ty=%lf\n",_Location->MiddlePoint[_CurrentTarget].x,_Location->MiddlePoint[_CurrentTarget].y);
            printf("VX=%f\tVY=%f\n",v_x,v_y);
            printf("state:back\tSchedule=%d\n==========================\n",_CurrentTarget);
            break;
        case finish:
            break;
        case halt:
            break;
        case error:
            break;
        default:                // ERROR SIGNAL
            printf("ERROR STATE\n");
            exit(FAULTEXECUTING);
    }
    _Env->Robot.v_x = v_x;
    _Env->Robot.v_y = v_y;
}