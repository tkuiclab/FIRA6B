#include <stdio.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include "../common/Env.h"



//攻擊防守,防守策略？
//沒有持球攻擊防守的策略
//
#define BallRadius 0.1055
#define CarRadius 0.34




class FIRA_behavior_class
{
//public:
//    enum State{
////        Initial,
////        Chase,
////        Attack,
////        U_Chase,
////        S_Attack,
////        SideSpeedUp
//        ////===============Behavior Attack=======================
//        Init_Attack,
//        Chase,
//        Attack,
//        U_Chase,
//        S_Attack,
//        SideSpeedUp,
//        Zone_Attack,
//        ////===============Behavior CornerKick================
//        CornerKick
//    };


private:
    int action;
    int state_attack[3];
    int state_cornerkick[3];

    // ========== test change roleAry[ i ] to escape from cornerkick and try to reset the state  begin ==========
    bool EscapeCornerKick[3] = {false,false,false};
    std::vector<double> Attack_Strategy;
    std::vector<double> Chase_Strategy;
    std::vector<double> Corner_Kick;
    std::vector<double> Side_Speed_UP;
    std::vector<double> TypeS_Attack;
    std::vector<double> TypeU_Chase;
    std::vector<double> Zone_Attack;
    std::vector<double> Dorsad_Attack;

    std::vector<int> StrategySelection;
    // ========== test change roleAry[ i ] to escape from cornerkick and try to reset the state  end ==========


//    State state_attack = Init_Attack;
//    State state_cornerkick = CornerKick;
    void ActionChase();
    //start---simulator---
	bool opponent;
    Environment env;
//    int roleAry[PLAYERS_PER_SIDE];
    int actionAry[PLAYERS_PER_SIDE];
    //end  ---simulator---
    int mTeam;
    int IsChase(int);
    int IsType_S(int);
    int IstypeU_Attack(int);

    //start---utility---
    double head2Obj(Vector3D robot,Vector3D dst,double robotRot);
    double vecAngle(Vector2d a,Vector2d b);
    //end---utility---
//    void StateInitAttack(int);
//    void StateAttack(int);
//    void StateType_SAttack(int);
//    void StateChase(int);
//    void StateType_UChase(int);
//    void StateSideSpeedUp(int);
//    void StateCornerKick(int);
//    void StateZoneAttack(int);
    // fuzzy state sys
    void StateCornerKick(int);
    void StateInitAttack(int);
    void StateAttack(int);
    void StateType_SAttack(int);
    void StateChase(int);
    void StateType_UChase(int);
    void StateSideSpeedUp(int);
    void StateZoneAttack(int);
//    void StateCornerKick(int);
    // fuzzy control sys
//    float fuzzy_controller(float, float, float, float);
//    float compute_grade1(float ,int);
//    float compute_grade2(float ,int);
//    float compute_grade3(float ,int);
//    float compute_grade4(float ,int);

    float error[2];
    bool decide_actionAttack;


    float MIN(float i,float j,float k,float l){
        if(i<=j&&i<=k&&i<=l) return i;
        else if(j<=i&&j<=k&&j<=l) return j;
        else if(k<=i&&k<=j&&k<=l) return k;
        else return l;
    }
    float MAX(float i,float j,float k,float l){
        if(i>=j&&i>=k&&i>=l) return i;
        else if(j>=i&&j>=k&&j>=l) return j;
        else if(k>=i&&k>=j&&k>=l) return k;
        else return l;
    }

public:
    //start---simulator---
    void setOpponent(bool iBool){opponent = iBool;}
    void setTeam(int team){mTeam = team;}
    void setEnv(Environment iEnv);
    Environment* getEnv(){return &env;}
    //end  ---simulator---
    FIRA_behavior_class();
    void behavior();

//    void actionchoise(int, int);

//--------------各種strategy case-------------------
    void behavior_Goalkeeper(int);
    void behavior_Attack(int);
    void behavior_Support(int);
    void behavior_Halt(int);
    void behavior_PenaltyKick(int);
    void behavior_ThrowIn(int);
    void behavior_CornerKick(int);
    void behavior_AvoidBarrier(int);
//--------------------------------------------------
    void readroleAry(int, int);
    int* getactionAry();

//    void teamStrategy();
    void loadParam(ros::NodeHandle *n);



};



