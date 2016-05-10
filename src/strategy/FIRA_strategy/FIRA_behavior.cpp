#include "FIRA_behavior.h"
#include "math.h"



FIRA_behavior_class::FIRA_behavior_class(){
   opponent = false;
}

//start---simulator---
void FIRA_behavior_class::setEnv(Environment iEnv){
//void FIRA_behavior_class::setEnv(){
   //Vector3D ball = iEnv.currentBall.pos;
   //printf("global_env   b_x=%lf,b_y=%lf\n",ball.x,ball.y);
   env = iEnv;

   if(opponent){
       for(int i = 0;i < PLAYERS_PER_SIDE;i++){
           env.home[i] = iEnv.opponent[i];
           env.opponent[i] = iEnv.home[i];
//           printf("in opponent behavior\n");
       }

   }
}

int FIRA_behavior_class::IsChase(int r_number){
        double distance_br = env.home[r_number].ball.distance;
        double angle_br = env.home[r_number].ball.angle;
        double angle_dr = env.home[r_number].goal.angle;
//        if(angle_br>180)angle_br-=360;
//        else if(angle_br<-180)angle_br+=360;
//        if(angle_dr>180)angle_dr-=360;
//        else if(angle_dr<-180)angle_dr+=360;
        ////nomorlization
        double alpha = angle_dr - angle_br;
        if(fabs(alpha)<=5 && distance_br<=0.33)
            return 0;
        else
            return 1;
}

int FIRA_behavior_class::IsType_S(int r_number){
    double distance_dr = env.home[r_number].goal.distance;
    if(distance_dr>=5 || distance_dr<=1.5)
        return 0;
    else
        return 1;
}

int FIRA_behavior_class::IstypeU_Attack(int r_number){
        double angle_br = env.home[r_number].ball.angle;
        double angle_dr = env.home[r_number].goal.angle;
        double alpha = angle_dr-angle_br;
        if(fabs(alpha)>90)
            return 1;
        else
            return 0;

}

//void FIRA_behavior_class::behavior(){
//    readroleAry();
//    getactionAry();

//}
void FIRA_behavior_class::readroleAry(int robotIndex,int role){
//    for(int i=0; i<PLAYERS_PER_SIDE;i++){
        switch(role){
            case Role_Goalkeeper:
                behavior_Goalkeeper(robotIndex);
                break;
            case Role_Attack:
                behavior_Attack(robotIndex);
                break;
            case Role_Support:
                behavior_Support(robotIndex);
                break;
            case Role_Halt:
                behavior_Halt(robotIndex);
                break;
            case Role_AvoidBarrier:
                behavior_AvoidBarrier(robotIndex);
//            case Role_PenaltyKick:
//                strategy_PenaltyKick(robotIndex);
//                break;
//            case Role_ThrowIn:
//                strategy_ThrowIn(robotIndex);
//                break;
//            case Role_CornerKick:
//                strategy_CornerKick(robotIndex);
//                break;
        }
//    }
}



int* FIRA_behavior_class::getactionAry(){
    return actionAry;
}
void FIRA_behavior_class::behavior_Goalkeeper(int robotIndex){
    actionAry[robotIndex] = action_Goalkeeper;
//    printf("behavoir_goalkeeper\n");
}

void FIRA_behavior_class::behavior_Attack(int robotIndex){
    if(IsChase(robotIndex)){
        if (IstypeU_Attack(robotIndex)){
            actionAry[robotIndex] = action_typeU_Attack;
//            printf("action_DirectChase\n");
        }
        else{
           actionAry[robotIndex] = action_Chase;
//           printf("action_Chase\n");
        }
    }
    else{
        if (IsType_S(robotIndex)){
            actionAry[robotIndex] =action_typeS_attack ;
//            printf("action_typeS_attack\n");

        }
        else{
            actionAry[robotIndex] =action_Attack ;
//            printf("action_Attack\n");
        }
    }
}

void FIRA_behavior_class::behavior_Support(int robotIndex){
    actionAry[robotIndex] = action_Support;
//    printf("behavoir_suport\n");
}
void FIRA_behavior_class::behavior_Halt(int robotIndex){
    actionAry[robotIndex] = action_Halt;
//    printf("behavoir_suport\n");
}
void FIRA_behavior_class::behavior_AvoidBarrier(int robotIndex){
    actionAry[robotIndex] = action_AvoidBarrier;
//    printf("behavoir_goalkeeper\n");
}
