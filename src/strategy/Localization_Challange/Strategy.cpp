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
    static int flag_chase = TRUE;
    int lost_ball_dis = 0.35;
    int lost_ball_angle = 5;
    static double Begin_time = ros::Time::now().toSec();// init timer begin
    double Current_time = ros::Time::now().toSec();   // init timer end
    double v_x,v_y,v_yaw;
    double accelerate = 1;
    double slow_factor =1;
    static int IMU_state = 0;
    static int last_state = 0;
    double compensation_distance = 0.1;
    double compensation_angle = ((int)imu+90+180)%360;
    double compensation_x = compensation_distance*cos(compensation_angle*DEG2RAD);
    double compensation_y = compensation_distance*sin(compensation_angle*DEG2RAD);
    if(flag)
        Begin_time = Current_time;
    if(Current_time-Begin_time < accelerate)
        slow_factor = exp(-2.1+2*((Current_time-Begin_time)/accelerate));
    else
        slow_factor = 1;
    if(_Env->Robot.ball.distance > lost_ball_dis || fabs(_Env->Robot.ball.angle) > lost_ball_angle){   
        if(flag_chase){
            last_state = _LocationState;
            _LocationState = chase;                      //  Check lost ball or not
            flag_chase = FALSE;
        }
    }else{
        _LocationState = last_state;
        flag_chase = TRUE;
    }
    switch(_LocationState){
        case forward:                                // Move to target poitn
            flag = FALSE;
            v_x = (_Location->LocationPoint[_CurrentTarget].x+compensation_x) - Robot.pos.x;
            v_y = (_Location->LocationPoint[_CurrentTarget].y+compensation_y) - Robot.pos.y;
            if(fabs(v_x)<=0.05 && fabs(v_y)<=0.05){
                _LocationState = back;
                flag = TRUE;
            }
            break;
        case back:                                   // Back to middle circle
            flag = FALSE;
            v_x = (_Location->MiddlePoint[_CurrentTarget].x+compensation_x) - Robot.pos.x;
            v_y = (_Location->MiddlePoint[_CurrentTarget].y+compensation_y) - Robot.pos.y;
            if(fabs(v_x)<=0.05 && fabs(v_y)<=0.05){
                if(_CurrentTarget==4)
                    _LocationState = finish;
                else{
                    _LocationState = forward;
                    flag = TRUE;
                }
            _CurrentTarget++;}
            break;  
        case finish:                                // Finish localization challange
            v_x = 0;
            v_y = 0;
            break;
        case chase:                                  // When robot lost the ball
            v_x = _Env->Robot.ball.distance * cos(_Env->Robot.ball.angle*DEG2RAD);
            v_y = _Env->Robot.ball.distance * cos(_Env->Robot.ball.angle*DEG2RAD);
            v_yaw = _Env->Robot.ball.angle;
            break;
        case error:
            printf("ERROR STATE\n");
            exit(FAULTEXECUTING);
            break;
        default:                // ERROR SIGNAL
            printf("ERROR STATE\n");
            exit(FAULTEXECUTING);
    }
    last_state = _LocationState;
    if(_LocationState != chase){
        if(imu>5)
            IMU_state = 1;
        else if(imu<-5)
            IMU_state = 2;
        else if(fabs(imu)<3)
            IMU_state = 0;
        else
            ;//do nothing
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
    }
    showInfo();
    _Env->Robot.v_x = v_x*slow_factor;
    _Env->Robot.v_y = v_y*slow_factor;
    _Env->Robot.v_yaw = v_yaw;
}
void Strategy::showInfo(){
    std::string Sv_x = "→";
    std::string Sv_y = "↑";
    std::string Sv_yaw = "↶";
    if(_Env->Robot.v_x>0)
        Sv_x = "→ ";
    else if(_Env->Robot.v_x<0)
        Sv_x = "← ";
    else   
        Sv_x = "";
    if(_Env->Robot.v_y>0)
        Sv_y = "↑ ";
    else if(_Env->Robot.v_y<0)
        Sv_y = "↓ ";
    else   
        Sv_y = "";
    if(_Env->Robot.v_yaw>0)
        Sv_yaw = "↶";
    else if(_Env->Robot.v_yaw<0)
        Sv_yaw = "↷";
    else   
        Sv_yaw = "";
    printf("***********************************************\n");
    printf("*                  START                      *\n");
    printf("***********************************************\n");
    switch(_LocationState){
    case forward:
        printf("Target : Point %d\t",_CurrentTarget);
        printf("State : Forward\n");break;
    case back:
        printf("Target : Point %d\t",_CurrentTarget);
        printf("State : Back\n");break;
    case finish:printf("State : Fisish\n");break;
    case chase:
        printf("Target : Ball\t");
        printf("State : Chase\n");break;
    case error:printf("State : Error\n");break;
    }
    if(_LocationState == forward)
        std::cout << "Target position : (" << _Location->LocationPoint[_CurrentTarget].x\
        << "," << _Location->LocationPoint[_CurrentTarget].y << ")\n";
    else if(_LocationState == back)
        std::cout << "Target position : (" << _Location->MiddlePoint[_CurrentTarget].x\
        << "," << _Location->MiddlePoint[_CurrentTarget].y << ")\n";
    else if (_LocationState == chase)
        std::cout << "Target position : (" << _Env->Robot.pos.x + \
        _Env->Robot.ball.distance*cos((_Env->Robot.pos.angle+_Env->Robot.ball.angle+90)*DEG2RAD)\
        << "," << _Env->Robot.pos.y + \
        _Env->Robot.ball.distance*sin((_Env->Robot.pos.angle+_Env->Robot.ball.angle+90)*DEG2RAD)\
        << ")" << std::endl;
    std::cout << "Robot position : (" << _Env->Robot.pos.x << "," << _Env->Robot.pos.y << ")\n";
    std::string haha = Sv_x+Sv_y+Sv_yaw;
    std::cout << "Direction : " << Sv_x+Sv_y+Sv_yaw << std::endl;
    std::cout << "Speed : (" << std::fixed << std::setprecision(2)\
    << fabs(_Env->Robot.v_x) << "," << fabs(_Env->Robot.v_y) << ","\
    << fabs(_Env->Robot.v_yaw) << ")" << std::endl;
    printf("==================== END ======================\n\n");
}