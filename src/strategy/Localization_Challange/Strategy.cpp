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
    _LocationState = turn;
    _CurrentTarget = 0;
    _Param->Strategy.HoldBall_Condition.resize(10);
    _Location = new LocationStruct;
    _Env = new Environment;
}
void Strategy::setParam(Parameter *Param){
    _Param = Param;
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
    // ROS_INFO("hold_angle=%lf\n",_Param->Strategy.HoldBall_Condition[0]);
    RobotData Robot;
    Robot.pos.x = _Env->Robot.pos.x;
    Robot.pos.y = _Env->Robot.pos.y;
    double imu = _Env->Robot.pos.angle;
    static int flag = TRUE;
    static int flag_chase = TRUE;
    double lost_ball_dis = _Param->Strategy.HoldBall_Condition[3];
    double lost_ball_angle = _Param->Strategy.HoldBall_Condition[2];
    double hold_ball_dis = _Param->Strategy.HoldBall_Condition[1];
    double hold_ball_angle = _Param->Strategy.HoldBall_Condition[0];
    static double Begin_time = ros::Time::now().toSec();// init timer begin
    double Current_time = ros::Time::now().toSec();   // init timer end
    double v_x,v_y,v_yaw;
    double accelerate = 1;
    double slow_factor =1;
    static int IMU_state = 0;
    static int last_state = turn;
    double compensation_distance = 0.1;
    double compensation_angle = ((int)imu+90+180)%360;
    double compensation_x = compensation_distance*cos(compensation_angle*DEG2RAD);
    double compensation_y = compensation_distance*sin(compensation_angle*DEG2RAD);
    Robot.ball.x = _Env->Robot.ball.distance*cos((_Env->Robot.pos.angle+_Env->Robot.ball.angle+90)*DEG2RAD);
    Robot.ball.y = _Env->Robot.ball.distance*sin((_Env->Robot.pos.angle+_Env->Robot.ball.angle+90)*DEG2RAD);
    Vector3D vector_turn,vector_br,vector_tr;
    double turn_yaw;
    vector_br.x = Robot.ball.x - Robot.pos.x;
    vector_br.y = Robot.ball.y - Robot.pos.y;
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
    }else if(_Env->Robot.ball.distance < hold_ball_dis && fabs(_Env->Robot.ball.angle) < hold_ball_angle){
        _LocationState = last_state;
        flag_chase = TRUE;
    }
    switch(_LocationState){
        case forward:                                // Move to target poitn
            last_state = _LocationState;
            flag = FALSE;
            v_x = (_Location->LocationPoint[_CurrentTarget].x+compensation_x) - Robot.pos.x;
            v_y = (_Location->LocationPoint[_CurrentTarget].y+compensation_y) - Robot.pos.y;
            if(fabs(v_x)<=0.05 && fabs(v_y)<=0.05){
                _LocationState = back;
                flag = TRUE;
            }
            break;
        case back:                                   // Back to middle circle
            last_state = _LocationState;
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
        case turn:
            if(last_state == forward){
                vector_tr.x = _Location->LocationPoint[_CurrentTarget].x - Robot.ball.x;
                vector_tr.y = _Location->LocationPoint[_CurrentTarget].y - Robot.ball.y;
            }else if(last_state == back){
                vector_tr.x = _Location->MiddlePoint[_CurrentTarget].x - Robot.ball.x;
                vector_tr.y = _Location->MiddlePoint[_CurrentTarget].y - Robot.ball.y;
            }else if(last_state == turn){
                vector_tr.x = _Location->LocationPoint[_CurrentTarget].x - Robot.ball.x;
                vector_tr.y = _Location->LocationPoint[_CurrentTarget].y - Robot.ball.y;
            }
            vector_tr.yaw = atan2(vector_tr.y,vector_tr.x)*RAD2DEG - (imu+90);
            if(vector_tr.yaw>0){
                vector_turn.x = cos(-90*DEG2RAD)*vector_br.x - sin(-90*DEG2RAD)*vector_br.y;
                vector_turn.y = sin(-90*DEG2RAD)*vector_br.x + cos(-90*DEG2RAD)*vector_br.y;
            }else{
                vector_turn.x = cos(90*DEG2RAD)*vector_br.x - sin(90*DEG2RAD)*vector_br.y;
                vector_turn.y = sin(90*DEG2RAD)*vector_br.x + cos(90*DEG2RAD)*vector_br.y;
            }
            v_x = vector_turn.x;
            v_y = vector_turn.y;
            v_yaw = vector_tr.yaw;
            slow_factor = 1;
            if(fabs(v_yaw) <= 3){
                if(last_state == forward){
                    _LocationState = back;
                }else if(last_state == back){
                    _LocationState = forward;
                }else if(last_state == turn){
                    _LocationState = forward;
                }
            }
            break;
        case error:
            printf("ERROR STATE\n");
            exit(FAULTEXECUTING);
            break;
        default:                // ERROR SIGNAL
            printf("ERROR STATE\n");
            exit(FAULTEXECUTING);
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
    if(_Env->Robot.v_x>0.1)
        Sv_x = "→ ";
    else if(_Env->Robot.v_x<-0.1)
        Sv_x = "← ";
    else   
        Sv_x = "";
    if(_Env->Robot.v_y>0.1)
        Sv_y = "↑ ";
    else if(_Env->Robot.v_y<-0.1)
        Sv_y = "↓ ";
    else   
        Sv_y = "";
    if(_Env->Robot.v_yaw>2)
        Sv_yaw = "↶";
    else if(_Env->Robot.v_yaw<-2)
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
    case turn:
        printf("Target : Turn\t");
        printf("State : Turn\n");break;
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
    printf("Speed : (%3f,%3f,%3f)\n",_Env->Robot.v_x,_Env->Robot.v_y,_Env->Robot.v_yaw);
    printf("==================== END ======================\n\n");
}