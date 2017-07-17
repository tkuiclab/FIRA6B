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
    for(int i=0;i<5;i++){
        printf("%d_x.%lf\n_",i,_Location->LocationPoint[i].x);
        printf("%d_y.%lf\n_",i,_Location->LocationPoint[i].y);
        printf("Middle %d_x.%lf\n_",i,_Location->MiddlePoint[i].x);
        printf("Middle %d_y.%lf\n_",i,_Location->LocationPoint[i].y);       
    }
    switch(_LocationState){
        case 0:                 // Move to target poitn
            // _Env->Robot.v_x = ;
            // _Env->Robot.v_y = ;
            break;
        case 1:                 // Back to middle circle
            // _Env->Robot.v_x = MidPoint[point].x;
            // _Env->Robot.v_y = MidPoint[point].y;
            break;
        default:                // ERROR SIGNAL
            printf("ERROR STATE\n");
            exit(FAULTEXECUTING);
    }
}