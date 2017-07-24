#include "FIRA_teamStrategy.h"
#include "math.h"
#define counter_delay 3
#define change_charactor_dis 0.1
static int shoot=1;


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
            break;
        case GameState_Halt:
            role_Halt();
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


    if(distance_dr1>=distance_dr2){
        if((angle_bd1<angle_bd2)&&(distance_br1<1)&&(angle_bd2>70)){
            printf("roleAry[%d]=Role_Attack\t",1);
            printf("roleAry[%d]=Role_Support\n",2);
        }else if((angle_bd2<angle_bd1)&&(distance_br2<1)&&(angle_bd1>70)){
            printf("roleAry[%d]=Role_Support\t",1);
            printf("roleAry[%d]=Role_Attack\n",2);
        }else{
            printf("roleAry[%d]=Role_Support\t",1);
            printf("roleAry[%d]=Role_Attack\n",2);
        }
    }else if(distance_dr1<distance_dr2){
        if((angle_bd1<angle_bd2)&&(distance_br1<1)&&(angle_bd2>70)){
            printf("roleAry[%d]=Role_Attack\t",1);
            printf("roleAry[%d]=Role_Support\n",2);
        }else if((angle_bd2<angle_bd1)&&(distance_br2<1)&&(angle_bd1>70)){
            printf("roleAry[%d]=Role_Support\t",1);
            printf("roleAry[%d]=Role_Attack\n",2);
        }else{
            printf("roleAry[%d]=Role_Attack\t",1);
            printf("roleAry[%d]=Role_Support\n",2);
        }
    }
    roleAry[0]=Role_Goalkeeper;
    roleAry[1]=Role_Attack;
    roleAry[2]=Role_Support;
}


void FIRA_teamStrategy_class::role_Halt(){
    roleAry[0] = Role_Halt;
    roleAry[1] = Role_Halt;
    roleAry[2] = Role_Halt;
    Begin_time = ros::Time::now().toSec();
    Current_time = ros::Time::now().toSec();
}

void FIRA_teamStrategy_class::role_FreeKick(){

}

void FIRA_teamStrategy_class::role_PenaltyKick(){
    roleAry[1] = Role_PenaltyKick;
    roleAry[2] = Role_PenaltyKick;
}

void FIRA_teamStrategy_class::role_FreeBall(){
    roleAry[0]=Role_Goalkeeper;
    roleAry[1]=Role_Halt;
    roleAry[2]=Role_Halt;
    if(Current_time - Begin_time<3){
        roleAry[1]=Role_Attack;
        roleAry[2]=Role_Support;
        //  寫死一隻SUP一隻ATK
    }else{
        //  之前的團側判斷
        //   printf("AttackerIs r[%d]\n",env.AttackerIs);
        printf("Another robot is Get Ball = %d\n",env.AnotherGetBall);
        printf("global_env->RobotNumber=%d\n",env.RobotNumber);
        double angle_chase = Chase_Strategy[3];//16.5
        double distance_chase = Chase_Strategy[4];//0.4
        printf("1\n");
        double distance_br=env.home[env.RobotNumber].ball.distance;//this robot
        double distance_dr=env.home[env.RobotNumber].goal.distance;
        double angle_br=env.home[env.RobotNumber].ball.angle;//this robot
        printf("1\n");
        static int attacker_select=0;
        static int this_attacker_select_counter=0;
        static int another_attacker_select_counter=0;
        static int two_robot_get_ball_counter=0;
        switch(attacker_select){
            case 0:// lost ball state
                printf("lost ball state\n");
                roleAry[env.RobotNumber]=Role_Attack;
                roleAry[env.AnotherRobotNumber]=Role_Attack;
                if((distance_br<=distance_chase)&&(fabs(angle_br)<=angle_chase)){//this robot catch ball
                    this_attacker_select_counter++;
                    printf("this_attacker_select_counter=%d\n",this_attacker_select_counter);
                    if(this_attacker_select_counter>=counter_delay){
                        attacker_select=1;
                        this_attacker_select_counter=0;
                        another_attacker_select_counter=0;
                    }
                }else{
                    this_attacker_select_counter=0;
                }

                if(env.AnotherGetBall==1){//this robot catch ball
                    another_attacker_select_counter++;
                    printf("another_attacker_select_counter=%d\n",another_attacker_select_counter);
                    if(another_attacker_select_counter>=counter_delay){
                        attacker_select=2;
                        this_attacker_select_counter=0;
                        another_attacker_select_counter=0;
                    }
                }else{
                    another_attacker_select_counter=0;
                }
            break;
            case 1:// this robot catch ball state
                printf("case 1\n");
                printf("r%d catch ball\n",env.RobotNumber);
                roleAry[env.RobotNumber]=Role_Attack;
                roleAry[env.AnotherRobotNumber]=Role_NewSupport;
                if((distance_br>distance_chase)||(fabs(angle_br)>angle_chase)){
                    this_attacker_select_counter++;
                    if(this_attacker_select_counter>=counter_delay){
                        attacker_select=0;
                        this_attacker_select_counter=0;
                        another_attacker_select_counter=0;
                    }
                }else{
                    this_attacker_select_counter=0;
                }
            break;
            case 2:// another robot catch ball state
                printf("case 2\n");
                printf("r%d catch ball\n",env.AnotherRobotNumber);
                roleAry[env.RobotNumber]=Role_NewSupport;
                roleAry[env.AnotherRobotNumber]=Role_Attack;
                if(env.AnotherGetBall==0){
                    another_attacker_select_counter++;
                    if(another_attacker_select_counter>=counter_delay){
                        attacker_select=0;
                        this_attacker_select_counter=0;
                        another_attacker_select_counter=0;
                    }
                }else{
                    another_attacker_select_counter=0;
                }
            break;
        }


        if(((distance_br<=distance_chase)&&(fabs(angle_br)<=angle_chase))&&(env.AnotherGetBall==1)){//means two robot get ball bug
            two_robot_get_ball_counter++;
            printf("two_robot_get_ball_counter=%d\n",two_robot_get_ball_counter);
            if(two_robot_get_ball_counter>=counter_delay){
                if(distance_dr<env.AnotherGoalDistance){//this robot attack,case 1
                    attacker_select=1;
                    this_attacker_select_counter=0;
                    another_attacker_select_counter=0;
                    two_robot_get_ball_counter=0;
                }else{// this robot support
                    attacker_select=2;
                    this_attacker_select_counter=0;
                    another_attacker_select_counter=0;
                    two_robot_get_ball_counter=0;
                }
            }
        }else{
            two_robot_get_ball_counter=0;
        }
        printf("r[%d] ball dis:%f\n",env.RobotNumber,env.home[env.RobotNumber].ball.distance);
        printf("r[%d] ball dis:%f\n",env.AnotherRobotNumber,env.AnotherBallDistance);
    }
    Current_time = ros::Time::now().toSec(); // 更新時間
}

void FIRA_teamStrategy_class::role_ThrowIn(){

    Vector3D goal;
    static int shoot=1;

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

    double distance_br1=env.home[1].ball.distance;
    double angle_br1=env.home[1].ball.angle;

    roleAry[1] = Role_Halt;
    roleAry[2] = Role_Halt;
    printf("shoot=%d\n",shoot);
    printf("distance=%f, angle_br1=%f\n",distance_br1, angle_br1);
    if(shoot == 1){
        if(distance_br1 < 0.4 && fabs(angle_br1) < 6){
            roleAry[1] = Role_ThrowIn;
            printf("catch\n");
            shoot--;
        }
    }else{
        role_Play();
    }
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
    if(n->getParam("/FIRA_Behavior/Chase_Strategy", Chase_Strategy)){
//        for(int i=0;i<5;i++)
//            std::cout<< "param Chase_Strategy["<< i << "]=" << Chase_Strategy[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
}

