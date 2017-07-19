#include "FIRA_teamStrategy.h"
#include "math.h"


FIRA_teamStrategy_class::FIRA_teamStrategy_class(){
   opponent = false;

   roleAry[0] = Role_Goalkeeper;
}

//start---simulator---
void FIRA_teamStrategy_class::setEnv(Environment iEnv){
   env = iEnv;

   if(opponent){
       for(int i = 0;i < PLAYERS_PER_SIDE;i++){
           env.home[i] = iEnv.opponent[i];
           env.opponent[i] = iEnv.home[i];
       }

   }
}
//###################################################//
//                                                   //
//                      Coach                        //
//                                                   //
//###################################################//
void FIRA_teamStrategy_class::teamStrategy(){
     switch(env.gameState){
        case GameState_Play:
            role_Play();
//            printf("start=======================\n");
            break;
        case GameState_Halt:
            role_Halt();
//            printf("stop=========================\n");
            break;
        case GameState_FreeKick:
            role_FreeKick();
            break;
        case GameState_PenaltyKick:
            role_PenaltyKick();
            break;
        case GameState_FreeBall:
            role_FreeBall();
            break;
        case GameState_ThrowIn:
            role_ThrowIn();
            break;
        case GameState_CornerKick:
            role_CornerKick();
            break;
        case GameState_GoalKick:
            role_GoalKick();
            break;
        case GameState_AvoidBarrier:
            role_AvoidBarrier();
            break;
    }
}

int* FIRA_teamStrategy_class::getRoleAry(){
    return roleAry;
}


//-------------------------Role Choose start-----------------------
void FIRA_teamStrategy_class::role_Play(){

    Vector3D goal ;

    if(env.teamcolor == "Blue")mTeam=Team_Blue;
    else if(env.teamcolor == "Yellow")mTeam=Team_Yellow;
    int goal_color;
    if(mTeam == Team_Blue){
        goal = env.yellow.pos;
        goal_color = Goal_Yellow;
        
    }else if (mTeam == Team_Yellow){
        goal = env.blue.pos;
        goal_color = Goal_Blue;
    }
    
    double distance_dr1=env.home[1].goal.distance;
    double distance_dr2=env.home[2].goal.distance;
    
    double distance_br1=env.home[1].ball.distance;
    double distance_br2=env.home[2].ball.distance;
    
    double angle_dr1=env.home[1].goal.angle;
    double angle_dr2=env.home[2].goal.angle;
    
    double angle_br1=env.home[1].ball.angle;
    double angle_br2=env.home[2].ball.angle;
    
    double angle_bd1=env.home[1].goal.angle-env.home[1].ball.angle;
    double angle_bd2=env.home[2].goal.angle-env.home[2].ball.angle;
    
    angle_bd1=fabs(angle_bd1);
    angle_bd2=fabs(angle_bd2);


//    if(distance_dr1>=distance_dr2){
//        if((angle_bd1<angle_bd2)&&(distance_br1<1)&&(angle_bd2>70)){
//            printf("roleAry[%d]=Role_Attack\t",1);
//            printf("roleAry[%d]=Role_Support\n",2);
//        }else if((angle_bd2<angle_bd1)&&(distance_br2<1)&&(angle_bd1>70)){
//            printf("roleAry[%d]=Role_Attack\t",2);
//            printf("roleAry[%d]=Role_Support\n",1);
//        }else{
//            printf("roleAry[%d]=Role_Attack\t",2);
//            printf("roleAry[%d]=Role_Support\n",1);
//        }
//    }else if(distance_dr1<distance_dr2){
//        if((angle_bd1<angle_bd2)&&(distance_br1<1)&&(angle_bd2>70)){
//            printf("roleAry[%d]=Role_Attack\t",1);
//            printf("roleAry[%d]=Role_Support\n",2);
//        }else if((angle_bd2<angle_bd1)&&(distance_br2<1)&&(angle_bd1>70)){
//            printf("roleAry[%d]=Role_Attack\t",2);
//            printf("roleAry[%d]=Role_Support\n",1);
//        }else{
//            printf("roleAry[%d]=Role_Attack\t",1);
//            printf("roleAry[%d]=Role_Support\n",2);
//        }
//    }
    roleAry[0]=Role_Goalkeeper;
    roleAry[1]=Role_Attack;
    roleAry[2]=Role_Attack;
}


void FIRA_teamStrategy_class::role_Halt(){
    roleAry[0] = Role_Halt;
    roleAry[1] = Role_Halt;
    roleAry[2] = Role_Halt;
}

void FIRA_teamStrategy_class::role_FreeKick(){

}

void FIRA_teamStrategy_class::role_PenaltyKick(){

}

void FIRA_teamStrategy_class::role_FreeBall(){

}

void FIRA_teamStrategy_class::role_ThrowIn(){
    roleAry[1] = Role_Halt;
    roleAry[2] = Role_Halt;
}

void FIRA_teamStrategy_class::role_CornerKick(){
    roleAry[0] = Role_CornerKick;
    roleAry[1] = Role_CornerKick;
    roleAry[2] = Role_CornerKick;
    printf("cornerKick\n");
}

void FIRA_teamStrategy_class::role_GoalKick(){

}

void FIRA_teamStrategy_class::role_AvoidBarrier(){
    roleAry[0] = Role_Halt;
    roleAry[1] = Role_AvoidBarrier;
    roleAry[2] = Role_Halt;
}

//---------------role choose end----------------
double FIRA_teamStrategy_class::vecAngle(Vector2d a,Vector2d b){

    a.normalize();
    b.normalize();
    double c = a.dot(b);
    double rad = acos(c);


    Vector3d a3d = Vector3d(a(0),a(1),0);
    Vector3d b3d = Vector3d(b(0),b(1),0);

    Vector3d tCross = a3d.cross(b3d);

    if(tCross(2) < 0)       return (-1)*rad*rad2deg;
    else                    return rad*rad2deg;
}
//==========for ROS special===============//
//###################################################//
//                                                   //
//                 load parameter                    //
//                                                   //
//###################################################//
void FIRA_teamStrategy_class::loadParam(ros::NodeHandle *n){

    std::string teamColor;

    if(n->getParam("/FIRA/teamColor",teamColor)){
        std::cout << "teamColor=" << teamColor <<std::endl;
    }

}

