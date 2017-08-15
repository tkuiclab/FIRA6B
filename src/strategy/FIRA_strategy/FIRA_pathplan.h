#include <stdio.h>
#include <iostream>
#include <math.h>
#include "../common/Env.h"
#include <ros/ros.h>

#define BallRadius 0.1055
#define CarRadius 0.34




class FIRA_pathplan_class
{
private:
    //start---simulator---
    bool opponent;
    Environment env;
    int mTeam;
    //end  ---simulator---

    //start---utility---
    double head2Obj(Vector3D robot,Vector3D dst,double robotRot);
    double vecAngle(Vector2d a,Vector2d b);
    //end---utility---


public:

    //start---simulator---
    void setOpponent(bool iBool){opponent = iBool;}
    void setTeam(int team){mTeam = team;}
    void setEnv(Environment iEnv);
    void shoot_init(){shoot = 0;}
    int getShoot(){return shoot;}
    Environment* getEnv(){return &env;}
    //end  ---simulator---

    FIRA_pathplan_class();
    void personalStrategy(int, int);
//    void strategy();

    void strategy_dst(double destination_x,double destination_y);
    void strategy_dst_head2ball(double destination_x,double destination_y);

//    void strategy_dontboom();


    void strategy_goalkeeper(int);
    void strategy_head2ball(int);

//--------------strategy case-------------------
    void strategy_Goalkeeper(int);
    void strategy_Attack(int);
    void strategy_typeS_Attack(int);
    void strategy_Zone_Attack(int);
    void strategy_typeU_Attack(int);
    void strategy_Dorsad_Attack(int);
    void strategy_Shoot_Attack(int);
    void strategy_Support(int);
    void strategy_Halt(int);
    void strategy_PenaltyKick(int);
    void strategy_ThrowIn(int);
    void strategy_CornerKick(int);
    void strategy_AvoidBarrier(int);
    void strategy_Chase(int);
    void strategy_Straight_Chase(int);
    void strategy_Slow_Chase(int);
    void strategy_KO5_Chase(int);
    void strategy_KO5_Attack(int);
    void strategy_SideSpeedUp(int);
    void strategy_Support_CatchBallState(int);
    void strategy_Support_LostBallState(int);
    void strategy_Support_Positioning(int);
    void strategy_Support_Test1(int);
    void strategy_Support_Test2(int);
    void strategy_Support_Test3(int);
    void strategy_MovetoYellowGate(int);
    void strategy_MovetoBlueGate(int);
    void strategy_LeaveBall(int);
    void strategy_LeaveLimitArea(int);
    void strategy_LeftRightMove(int);
    void strategy_invLeftRightMove(int);
    void strategy_Support_LostInternet(int);
    void strategy_MovetoGoal(int);
    void strategy_MovetoOpGoal(int);
    void strategy_MovetoGoalEdge1(int);
    void strategy_MovetoGoalEdge2(int);
    void strategy_MovetoOpGoalEdge1(int);
    void strategy_MovetoOpGoalEdge2(int);
    void strategy_Stop(int);
    void strategy_Block(int);
    void strategy_Kick(int);
    void strategy_FreeKick(int);
    void strategy_Escape_Attack(int);
    void strategy_Straight_Attack(int);
    void Movement_Forward(int);
    void Movement_Backward(int);
    void Movement_Left(int);
    void Movement_LeftForward(int);
    void Movement_LeftBackward(int);
    void Movement_Right(int);
    void Movement_RightForward(int);
    void Movement_RightBackward(int);



//--------------------------------------------------

//--------------role case-------------------
    void role_Play();
    void role_Halt();
    void role_FreeKick();
    void role_PenaltyKick();
    void role_FreeBall();
    void role_ThrowIn();
    void role_CornerKick();
    void role_GoalKick();
    void role_AvoidBarrier();
//--------------------------------------------------

//    void teamStrategy();

    //==========for ROS special===============//
    std::string teamColor;
    double beta_const = 0.9;
    double long_rush_alpha;
    double long_rush_dis_br;
    double long_rush_speed_const;
    double short_rush_dis_dr;
    double short_rush_alpha;
    double short_rush_dis_br;
    double short_rush_speed_const;
    double close_ball_dis_const;
    double close_ball_speed_const;
    double far_ball_speed_const;
    double head2ball_speed;
    double goalkeeper_radius;
    double goalkeeper_front_dis;
    double goalkeeper_mid_dis;
    double goalkeeper_side_dis;
    double goalkeeper_front_angle;
    double goalkeeper_mid_angle;
    double goalkeeper_front_speed;
    double goalkeeper_mid_speed;
    double goalkeeper_side_speed;

    std::vector<double> SPlanning_Velocity;
    std::vector<double> Distance_Settings;
    std::vector<double> Attack_Strategy;
    std::vector<double> Chase_Strategy;
    std::vector<double> Zone_Attack;
    std::vector<double> TypeS_Attack;
    std::vector<double> TypeU_Attack;
    std::vector<double> Side_Speed_Up;
    std::vector<double> Dorsad_Attack;
    std::vector<double> Corner_Kick;
    std::vector<double> Penalty_Kick;
    std::vector<int> Strategy_Selection;
    std::vector<double> General_PathPlan;


    // Robot shoot signal publisher
    int shoot = 0;

    void loadParam(ros::NodeHandle *n);
};



