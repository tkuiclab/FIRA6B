#include <stdio.h>
#include <iostream>
#include <math.h>
#include "../common/Env.h"
#include <ros/ros.h>
#include "std_msgs/Int32MultiArray.h"

class FIRA_teamStrategy_class
{
private:
    //start---simulator---
    bool opponent;
    Environment env;
    int mTeam;
    int roleAry[PLAYERS_PER_SIDE];
    int send_order;
    ros::Subscriber TeamColor;
    std::vector<double> Chase_Strategy;
    //end  ---simulator---
    double vecAngle(Vector2d a,Vector2d b);
    // int SupportCnt;
public:
    //start---simulator---
    void setOpponent(bool iBool){opponent = iBool;}
    void setTeam(int team){mTeam = team;}
    void setEnv(Environment iEnv);
    Environment* getEnv(){return &env;}
    //end  ---simulator---

    FIRA_teamStrategy_class();
    void teamStrategy();

//--------------各種role case-------------------
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
    int* getRoleAry();
    int* getOrder();
    //==========for ROS special===============//
    void loadParam(ros::NodeHandle *n);


};
