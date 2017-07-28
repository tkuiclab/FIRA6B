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
Strategy::Strategy()
{
    _LocationState = turn;
    _Last_state = turn;
    _CurrentTarget = 0;
    _Location = new LocationStruct;
    _Env = new Environment;
}
void Strategy::setParam(Parameter *Param)
{
    _Param = Param;
}
void Strategy::GameState(int int_GameState)
{
    switch (int_GameState)
    {
    case STATE_HALT:
        StrategyHalt();
        break;
    case STATE_LOCALIZATION:
        StrategyLocalization();
        break;
    }
}
void Strategy::StrategyHalt()
{
    _Env->Robot.v_x = 0;
    _Env->Robot.v_y = 0;
    _Env->Robot.v_yaw = 0;
}
void Strategy::StrategyLocalization()
{
    // ROS_INFO("hold_angle=%lf\n",_Param->Strategy.HoldBall_Condition[0]);
    RobotData Robot;
    Robot.pos.x = _Env->Robot.pos.x;
    Robot.pos.y = _Env->Robot.pos.y;
    double imu = _Env->Robot.pos.angle;
    double absolute_front = imu + 90;
    static int flag = TRUE;
    static int flag_chase = TRUE;
    double lost_ball_dis = _Param->Strategy.HoldBall_Condition[3];
    double lost_ball_angle = _Param->Strategy.HoldBall_Condition[2];
    double hold_ball_dis = _Param->Strategy.HoldBall_Condition[1];
    double hold_ball_angle = _Param->Strategy.HoldBall_Condition[0];
    static double Begin_time = ros::Time::now().toSec(); // init timer begin
    double Current_time = ros::Time::now().toSec();      // init timer end
    double v_x, v_y, v_yaw;
    double accelerate = 1;
    double slow_factor = 1;
    static int IMU_state = 0;
    double compensation_distance = 0.1;
    double compensation_angle = ((int)absolute_front + 180) % 360;
    double compensation_x = compensation_distance * cos(compensation_angle * DEG2RAD);
    double compensation_y = compensation_distance * sin(compensation_angle * DEG2RAD);
    Robot.ball.x = _Env->Robot.ball.distance * cos((_Env->Robot.pos.angle + _Env->Robot.ball.angle + 90) * DEG2RAD);
    Robot.ball.y = _Env->Robot.ball.distance * sin((_Env->Robot.pos.angle + _Env->Robot.ball.angle + 90) * DEG2RAD);
    Vector3D vector_turn, vector_br, vector_tr;
    double turn_yaw;
    vector_br.x = Robot.ball.x - Robot.pos.x;
    vector_br.y = Robot.ball.y - Robot.pos.y;
    if (flag)
        Begin_time = Current_time;
    if (Current_time - Begin_time < accelerate)
        slow_factor = exp(-2.1 + 2 * ((Current_time - Begin_time) / accelerate));
    else
        slow_factor = 1;
    //    if(_Env->Robot.ball.distance > lost_ball_dis || fabs(_Env->Robot.ball.angle) > lost_ball_angle){
    //        if(flag_chase){
    //            _Last_state = _LocationState;
    //            _LocationState = chase;                      //  Check lost ball or not
    //            flag_chase = FALSE;
    //        }
    //    }else if(_Env->Robot.ball.distance < hold_ball_dis && fabs(_Env->Robot.ball.angle) < hold_ball_angle){
    //        _LocationState = _Last_state;
    //        flag_chase = TRUE;
    //    }
    Normalization(absolute_front);
    switch (_LocationState)
    {
    case forward: // Move to target poitn
        _Last_state = _LocationState;
        flag = FALSE;
        v_x = (_Location->LocationPoint[_CurrentTarget].x + compensation_x) - Robot.pos.x;
        v_y = (_Location->LocationPoint[_CurrentTarget].y + compensation_y) - Robot.pos.y;
        double v_x_temp, v_y_temp;
        v_x_temp = v_x * cos((-imu) * DEG2RAD) - v_y * sin((-imu) * DEG2RAD);
        v_y_temp = v_x * sin((-imu) * DEG2RAD) + v_y * cos((-imu) * DEG2RAD);
        v_x = v_x_temp;
        v_y = v_y_temp;
        v_yaw = atan2(_Location->LocationPoint[_CurrentTarget].y + compensation_y, _Location->LocationPoint[_CurrentTarget].x + compensation_x) * RAD2DEG - absolute_front;
        Normalization(v_yaw);
        ROS_INFO("v_yaw = %lf!!!!!!", v_yaw);
        if (fabs(v_yaw) < 3)
            v_yaw = 0;
        if (fabs(v_x) <= 0.1 && fabs(v_y) <= 0.1)
        {
            _LocationState = turn;
            flag = TRUE;
        }
        break;
    case back: // Back to middle circle
        _Last_state = _LocationState;
        flag = FALSE;
        v_x = (_Location->MiddlePoint[_CurrentTarget].x + compensation_x) - Robot.pos.x;
        v_y = (_Location->MiddlePoint[_CurrentTarget].y + compensation_y) - Robot.pos.y;
        v_x_temp = v_x * cos((-imu) * DEG2RAD) - v_y * sin((-imu) * DEG2RAD);
        v_y_temp = v_x * sin((-imu) * DEG2RAD) + v_y * cos((-imu) * DEG2RAD);
        v_x = v_x_temp;
        v_y = v_y_temp;
        v_yaw = atan2(_Location->MiddlePoint[_CurrentTarget].y + compensation_y, _Location->MiddlePoint[_CurrentTarget].x + compensation_x) * RAD2DEG - absolute_front;
        if (fabs(v_x) <= 0.1 && fabs(v_y) <= 0.1)
        {
            if (_CurrentTarget == 4)
                _LocationState = finish;
            else
            {
                _LocationState = turn;
                flag = TRUE;
            }
            _CurrentTarget++;
        }
        break;
    case finish: // Finish localization challange
        v_x = 0;
        v_y = 0;
        break;
    case chase: // When robot lost the ball
        v_x = _Env->Robot.ball.distance * cos(_Env->Robot.ball.angle * DEG2RAD);
        v_y = _Env->Robot.ball.distance * cos(_Env->Robot.ball.angle * DEG2RAD);
        v_yaw = _Env->Robot.ball.angle;
        break;
    case turn:
        if (_Last_state == forward)
        {
            vector_tr.x = _Location->MiddlePoint[_CurrentTarget].x - Robot.pos.x;
            vector_tr.y = _Location->MiddlePoint[_CurrentTarget].y - Robot.pos.y;
        }
        else if (_Last_state == back)
        {
            vector_tr.x = _Location->LocationPoint[_CurrentTarget].x - Robot.pos.x;
            vector_tr.y = _Location->LocationPoint[_CurrentTarget].y - Robot.pos.y;
        }
        else if (_Last_state == turn)
        {
            vector_tr.x = _Location->LocationPoint[_CurrentTarget].x - Robot.pos.x;
            vector_tr.y = _Location->LocationPoint[_CurrentTarget].y - Robot.pos.y;
        }
        vector_tr.yaw = atan2(vector_tr.y, vector_tr.x) * RAD2DEG - absolute_front;
        v_x = 0;               // don't give it horizen velocity
        v_y = 100;             // full power
        v_yaw = vector_tr.yaw; // turn to target
        Normalization(v_yaw);
        ROS_INFO("v_yaw = %lf!!!!!!", v_yaw);
        if (fabs(v_yaw) <= 3)
        {
            if (_Last_state == forward)
            {
                ROS_INFO("change to back");
                _LocationState = back;
            }
            else if (_Last_state == back)
            {
                ROS_INFO("change to forward");
                _LocationState = forward;
            }
            else if (_Last_state == turn)
            {
                ROS_INFO("change to forward");
                _LocationState = forward;
            }
        }
        break;
    case error:
        printf("ERROR STATE\n");
        exit(FAULTEXECUTING);
        break;
    default: // ERROR SIGNAL
        printf("UNDEFINE STATE\n");
        exit(FAULTEXECUTING);
    }
       OptimatePath();
    // showInfo(imu, compensation_x, compensation_y);
    Normalization(v_yaw);
    _Env->Robot.v_x = v_x;
    _Env->Robot.v_y = v_y;
    _Env->Robot.v_yaw = v_yaw;
}
void Strategy::Forward(RobotData Robot, int &v_x, int &v_y, int flag)
{
    ;
}
void Strategy::Back(int flag)
{
    ;
}
void Strategy::Turn()
{
    ;
}
void Strategy::Chase()
{
    ;
}
void Strategy::OptimatePath()
{
    int horizon_point = -1;
    int hotizon_location = -1;
    for (int i = 0; i < 5; i++){
        if (_Location->LocationPoint[i].y == 0 || _Location->LocationPoint[i].x == 0)
        {
            if (_Location->LocationPoint[i].y > 0)
                hotizon_location = up;
            else if (_Location->LocationPoint[i].y < 0)
                hotizon_location = down;
            else if (_Location->LocationPoint[i].x > 0)
                hotizon_location = right;
            else if (_Location->LocationPoint[i].x < 0)
                hotizon_location = left;
        }
    }
    switch (hotizon_location)
    {
    case up:
        break;
    case down:
        break;
    case right:
        break;
    case left:
        break;
    default:
        printf("UNDEFINE STATE\n");
        exit(FAULTEXECUTING);
    }
    // for (int i = 0; i < 5; i++)
    //     for (int j = i + 1; j < 5; j++)
    //     {
    //         double Slope = (_Location->LocationPoint[i].y - _Location->LocationPoint[j].y) / (_Location->LocationPoint[i].x - _Location->LocationPoint[j].x);
    //         if (Slope > 999)
    //             Slope = 999;
    //         double dis = (_Location->LocationPoint[j].y - Slope * _Location->LocationPoint[j].x) / sqrt(Slope * Slope + 1);
    //         if (fabs(dis) < 0.5)
    //             printf("dis %d -> %d :  = %lf\n", i + 1, j + 1, dis);
    //         double angle = atan2(_Location->LocationPoint[i].y, _Location->LocationPoint[i].x) * RAD2DEG;
    //         double angle_next = atan2(_Location->LocationPoint[j].y, _Location->LocationPoint[j].x) * RAD2DEG;
    //         double angle_diff = angle_next - angle;
    //         Normalization(angle_diff);
    //         angle_diff = fabs(angle_diff);
    //         printf("angle_diff %d -> %d :  = %lf\n", i + 1, j + 1, angle_diff);
    //     }
    // printf("====================\n");
}
void Strategy::showInfo(double imu, double compensation_x, double compensation_y)
{
    std::string Sv_x = "→";
    std::string Sv_y = "↑";
    std::string Sv_yaw = "↶";
    if (_Env->Robot.v_x > 0.1)
        Sv_x = "→ ";
    else if (_Env->Robot.v_x < -0.1)
        Sv_x = "← ";
    else
        Sv_x = "";
    if (_Env->Robot.v_y > 0.1)
        Sv_y = "↑ ";
    else if (_Env->Robot.v_y < -0.1)
        Sv_y = "↓ ";
    else
        Sv_y = "";
    if (_Env->Robot.v_yaw > 2)
        Sv_yaw = "↶";
    else if (_Env->Robot.v_yaw < -2)
        Sv_yaw = "↷";
    else
        Sv_yaw = "";
    printf("***********************************************\n");
    printf("*                  START                      *\n");
    printf("***********************************************\n");
    switch (_LocationState)
    {
    case forward:
        printf("Target : Point %d\t", _CurrentTarget);
        printf("State : Forward\n");
        break;
    case back:
        printf("Target : Point %d\t", _CurrentTarget);
        printf("State : Back\n");
        break;
    case finish:
        printf("State : Fisish\n");
        break;
    case chase:
        printf("Target : Ball\t");
        printf("State : Chase\n");
        break;
    case turn:
        printf("Target : Point %d\t", _CurrentTarget);
        printf("State : Turn\n");
        break;
    case error:
        printf("State : Error\n");
        break;
    }
    if (_LocationState == forward)
        std::cout << "Target position : (" << _Location->LocationPoint[_CurrentTarget].x + compensation_x
                  << "," << _Location->LocationPoint[_CurrentTarget].y + compensation_y << ")\n";
    else if (_LocationState == back)
        std::cout << "Target position : (" << _Location->MiddlePoint[_CurrentTarget].x + compensation_x
                  << "," << _Location->MiddlePoint[_CurrentTarget].y + compensation_y << ")\n";
    else if (_LocationState == chase)
        std::cout << "Target position : (" << _Env->Robot.pos.x + _Env->Robot.ball.distance * cos((_Env->Robot.pos.angle + _Env->Robot.ball.angle + 90) * DEG2RAD)
                  << "," << _Env->Robot.pos.y + _Env->Robot.ball.distance * sin((_Env->Robot.pos.angle + _Env->Robot.ball.angle + 90) * DEG2RAD)
                  << ")" << std::endl;
    else if (_LocationState == turn)
        if (_Last_state == forward)
            std::cout << "Target position : (" << _Location->MiddlePoint[_CurrentTarget].x + compensation_x
                      << "," << _Location->MiddlePoint[_CurrentTarget].y + compensation_y << ")\n";
        else
            std::cout << "Target position : (" << _Location->MiddlePoint[_CurrentTarget].x + compensation_x
                      << "," << _Location->MiddlePoint[_CurrentTarget].y + compensation_y << ")\n";
    std::cout << "Imu = " << imu << std::endl;
    std::cout << "Robot position : (" << _Env->Robot.pos.x << "," << _Env->Robot.pos.y << ")\n";
    std::string haha = Sv_x + Sv_y + Sv_yaw;
    std::cout << "Direction : " << Sv_x + Sv_y + Sv_yaw << std::endl;
    printf("Speed : (%3f,%3f,%3f)\n", _Env->Robot.v_x, _Env->Robot.v_y, _Env->Robot.v_yaw);
    printf("==================== END ======================\n\n");
}
void Strategy::Normalization(double &angle)
{
    if (angle > 180)
        angle -= 360;
    else if (angle < -180)
        angle += 360;
}