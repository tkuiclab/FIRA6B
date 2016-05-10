#include <stdio.h>
#include <iostream>
#include <math.h>
#include "../common/Env.h"



//攻擊防守,防守策略？
//沒有持球攻擊防守的策略
//
#define BallRadius 0.1055
#define CarRadius 0.34




class FIRA_behavior_class
{
private:
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




};



