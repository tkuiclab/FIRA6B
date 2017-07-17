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
    _Env = new Environment;
}
void Strategy::setEnv(Environment *Env){
    _Env = Env;
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
    
    switch(_LocationState){
        case 0:                 // Move to target poitn
            // _Env->Robot.v_x = LocationPoint[point].x;
            // _Env->Robot.v_y = LocationPoint[point].y;
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