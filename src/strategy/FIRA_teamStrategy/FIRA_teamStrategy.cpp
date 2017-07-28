#include "FIRA_teamStrategy.h"
#include "math.h"
#define counter_delay 2
#define change_charactor_dis 0.1
static int shoot=1;
static double Begin_time = 0;
static double Current_time = 0;
static int last_gamestate= 0;
static int this_robot_role = 0; // 0 = attack, 1 = support
static int role_flag = 1;
static int shoot_flag = 1;

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
     if(last_gamestate!=env.gameState){// switch gamestate
         Begin_time = ros::Time::now().toSec();
         Current_time = ros::Time::now().toSec();
         role_flag = 1;
         shoot_flag = 1;
     }
     last_gamestate = env.gameState;
}

int* FIRA_teamStrategy_class::getRoleAry(){
    return roleAry;
}


//-------------------------Role Choose start-----------------------
void FIRA_teamStrategy_class::role_Play(){

    Vector3D goal ;
    double distance_br=env.home[env.RobotNumber].ball.distance;//this robot
    double distance_dr=env.home[env.RobotNumber].goal.distance;
    double angle_br=env.home[env.RobotNumber].ball.angle;//this robot
    double angle_chase = Chase_Strategy[3];//16.5
    double distance_chase = Chase_Strategy[4];//0.4

    if((distance_br<=distance_chase)&&(fabs(angle_br)<=angle_chase)){//if catch ball
        if(role_flag==1){
            this_robot_role = Role_Attack;
            role_flag--;
        }
    }else{//not catch ball
        if(role_flag==1){
            this_robot_role = Role_NewSupport;
            role_flag--;
        }
    }

    int attacker_select_starter;// teamstrategy starter character choose
    if(this_robot_role==Role_NewSupport){
        attacker_select_starter=2;
    }else if(this_robot_role==Role_Attack){
        attacker_select_starter=1;
    }
    
    if(env.isteamstrategy==1){//if teamstrategy open
        static int attacker_select = attacker_select_starter;
        static int this_attacker_select_counter=0;
        static int another_attacker_select_counter=0;
        static int two_robot_get_ball_counter=0;
        switch(attacker_select){
            case 0:// lost ball state
                printf("lost ball state\n");
                roleAry[env.RobotNumber]=Role_Attack;
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

                if(env.AnotherGoalDistance<distance_dr&&distance_dr<1.6){
                    roleAry[env.RobotNumber]=Role_NewSupport;
                }
            break;
            case 1:// this robot catch ball state
                printf("this robot catch ball\n");
                roleAry[env.RobotNumber]=Role_Attack;
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
                printf("another robot catch ball\n");
                roleAry[env.RobotNumber]=Role_NewSupport;
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
    }else{// no teamstrategy, two attack
        printf("single\n");
        roleAry[0]=Role_Goalkeeper;
        roleAry[1]=Role_Test1;
        roleAry[2]=Role_Attack;
    }


//    if(distance_dr1>=distance_dr2){
//        if((angle_bd1<angle_bd2)&&(distance_br1<1)&&(angle_bd2>70)){
//            printf("roleAry[%d]=Role_Attack\t",1);
//            printf("roleAry[%d]=Role_Support\n",2);
//        }else if((angle_bd2<angle_bd1)&&(distance_br2<1)&&(angle_bd1>70)){
//            printf("roleAry[%d]=Role_Support\t",1);
//            printf("roleAry[%d]=Role_Attack\n",2);
//        }else{
//            printf("roleAry[%d]=Role_Support\t",1);
//            printf("roleAry[%d]=Role_Attack\n",2);
//        }
//    }else if(distance_dr1<distance_dr2){
//        if((angle_bd1<angle_bd2)&&(distance_br1<1)&&(angle_bd2>70)){
//            printf("roleAry[%d]=Role_Attack\t",1);
//            printf("roleAry[%d]=Role_Support\n",2);
//        }else if((angle_bd2<angle_bd1)&&(distance_br2<1)&&(angle_bd1>70)){
//            printf("roleAry[%d]=Role_Support\t",1);
//            printf("roleAry[%d]=Role_Attack\n",2);
//        }else{
//            printf("roleAry[%d]=Role_Attack\t",1);
//            printf("roleAry[%d]=Role_Support\n",2);
//        }
//    }
    // ROS_INFO("hahaaha");
//    roleAry[0]=Role_Goalkeeper;
//    roleAry[1]=Role_Attack;
//    roleAry[2]=Role_Test1;
}


void FIRA_teamStrategy_class::role_Halt(){
    roleAry[0] = Role_Halt;
    roleAry[1] = Role_Halt;
    roleAry[2] = Role_Halt;
    Begin_time = ros::Time::now().toSec();
    Current_time = ros::Time::now().toSec();
    // counter = 0;
    // Begin_time = ros::Time::now().toSec();
    // Current_time = ros::Time::now().toSec();
}

void FIRA_teamStrategy_class::role_FreeKick(){
    printf("Free Kick\n");
    Current_time = ros::Time::now().toSec();
    printf("Current_time-Begin_time=%f\n",fabs(Current_time-Begin_time));
    double distance_br=env.home[env.RobotNumber].ball.distance;//this robot
    double distance_dr=env.home[env.RobotNumber].goal.distance;
    double angle_br=env.home[env.RobotNumber].ball.angle;//this robot
    double angle_chase = Chase_Strategy[3];//16.5
    double distance_chase = Chase_Strategy[4];//0.4
    //depend on if this robot catch ball, use role flag decide role
    if((distance_br<=distance_chase)&&(fabs(angle_br)<=angle_chase)){//if catch ball
        if(role_flag==1){
            this_robot_role = Role_NewSupport;
            role_flag--;
        }
    }else{//not catch ball
        if(role_flag==1){
            this_robot_role = Role_Attack;
            role_flag--;
        }
    }

    if(this_robot_role == Role_NewSupport){
        if(fabs(Current_time-Begin_time)<=1.5){
           roleAry[env.RobotNumber] = Role_FreeKick;// backward
        }else{
            if(shoot_flag==1){// is support and need shoot
                roleAry[env.RobotNumber] = Role_Kick;
                shoot_flag--;
            }else{// is support and don't need shoot
                roleAry[env.RobotNumber] = this_robot_role;
            }
        }
    }else{// is attacker
        if(fabs(Current_time-Begin_time)<=2){
           roleAry[env.RobotNumber] = Role_Halt;
        }else{
           roleAry[env.RobotNumber] = Role_Attack;
        }
    }

    int attacker_select_starter;// which role you are when 2sec, will continue on teamstrategy
    if(this_robot_role==Role_NewSupport){
        attacker_select_starter=2;
    }else if(this_robot_role==Role_Attack){
        attacker_select_starter=1;
    }
    if(env.isteamstrategy==1&&fabs(Current_time-Begin_time)>=2){//if teamstrategy open
        static int attacker_select = attacker_select_starter;
        static int this_attacker_select_counter=0;
        static int another_attacker_select_counter=0;
        static int two_robot_get_ball_counter=0;
        switch(attacker_select){
            case 0:// lost ball state
                printf("lost ball state\n");
                roleAry[env.RobotNumber]=Role_Attack;
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

                if(env.AnotherGoalDistance<distance_dr&&distance_dr<1.6){
                    roleAry[env.RobotNumber]=Role_NewSupport;
                }
            break;
            case 1:// this robot catch ball state
                printf("this robot catch ball\n");
                roleAry[env.RobotNumber]=Role_Attack;
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
                printf("another robot catch ball\n");
                roleAry[env.RobotNumber]=Role_NewSupport;
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
    }
}

void FIRA_teamStrategy_class::role_PenaltyKick(){
    printf("Penalty Kick\n");
    double distance_br=env.home[env.RobotNumber].ball.distance;//this robot
    double distance_dr=env.home[env.RobotNumber].goal.distance;
    double angle_br=env.home[env.RobotNumber].ball.angle;//this robot
    double angle_chase = Chase_Strategy[3];//16.5
    double distance_chase = Chase_Strategy[4];//0.4
    //depend on if this robot catch ball, use role flag decide role
    if((distance_br<=distance_chase)&&(fabs(angle_br)<=angle_chase)){//catch ball, the catch ball robot penaltykick
        roleAry[env.RobotNumber] = Role_PenaltyKick;
    }else{// not catch ball robot stand by
        roleAry[env.RobotNumber] = Role_Halt;
    }

}

void FIRA_teamStrategy_class::role_FreeBall(){
    printf("Free Ball\n");
    Current_time = ros::Time::now().toSec();
    printf("Current_time-Begin_time=%f\n",fabs(Current_time-Begin_time));
    double distance_br=env.home[env.RobotNumber].ball.distance;//this robot
    double distance_dr=env.home[env.RobotNumber].goal.distance;
    double angle_br=env.home[env.RobotNumber].ball.angle;//this robot
    double angle_chase = Chase_Strategy[3];//16.5
    double distance_chase = Chase_Strategy[4];//0.4
    //depend on if this robot catch ball, use role flag decide role
    if((distance_br<=distance_chase)&&(fabs(angle_br)<=angle_chase)){//if catch ball
        if(role_flag==1){
            this_robot_role = Role_NewSupport;
            role_flag--;
        }
    }else{//not catch ball
        if(role_flag==1){
            this_robot_role = Role_Attack;
            role_flag--;
        }
    }
    if(this_robot_role == Role_NewSupport){
        if(shoot_flag==1){// is support and need shoot
            roleAry[env.RobotNumber] = Role_Kick;
            shoot_flag--;
        }else{// is support and don't need shoot
            roleAry[env.RobotNumber] = this_robot_role;
        }
    }else{// is attacker
        roleAry[env.RobotNumber] = this_robot_role;
    }

    int attacker_select_starter;// which role you are when 2sec, will continue on teamstrategy
    if(this_robot_role==Role_NewSupport){
        attacker_select_starter=2;
    }else if(this_robot_role==Role_Attack){
        attacker_select_starter=1;
    }
    if(env.isteamstrategy==1&&fabs(Current_time-Begin_time)>=2){//if teamstrategy open
        static int attacker_select = attacker_select_starter;
        static int this_attacker_select_counter=0;
        static int another_attacker_select_counter=0;
        static int two_robot_get_ball_counter=0;
        switch(attacker_select){
            case 0:// lost ball state
                printf("lost ball state\n");
                roleAry[env.RobotNumber]=Role_Attack;
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

                if(env.AnotherGoalDistance<distance_dr&&distance_dr<1.6){
                    roleAry[env.RobotNumber]=Role_NewSupport;
                }         
            break;
            case 1:// this robot catch ball state
                printf("this robot catch ball\n");
                roleAry[env.RobotNumber]=Role_Attack;
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
                printf("another robot catch ball\n");
                roleAry[env.RobotNumber]=Role_NewSupport;
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
        printf("this_attacker_select_counter=%d\n",this_attacker_select_counter);
        printf("another_attacker_select_counter=%d\n",another_attacker_select_counter);
    }

}

void FIRA_teamStrategy_class::role_ThrowIn(){
    printf("Throw In\n");
    Current_time = ros::Time::now().toSec();
    printf("Current_time-Begin_time=%f\n",fabs(Current_time-Begin_time));
    double distance_br=env.home[env.RobotNumber].ball.distance;//this robot
    double distance_dr=env.home[env.RobotNumber].goal.distance;
    double angle_br=env.home[env.RobotNumber].ball.angle;//this robot
    double angle_chase = Chase_Strategy[3];//16.5
    double distance_chase = Chase_Strategy[4];//0.4
    //depend on if this robot catch ball, use role flag decide role
    if((distance_br<=distance_chase)&&(fabs(angle_br)<=angle_chase)){//if catch ball
        if(role_flag==1){
            this_robot_role = Role_NewSupport;
            role_flag--;
        }
    }else{//not catch ball
        if(role_flag==1){
            this_robot_role = Role_Attack;
            role_flag--;
        }
    }
    if(this_robot_role == Role_NewSupport){
        if(shoot_flag==1){// is support and need shoot
            roleAry[env.RobotNumber] = Role_Kick;
            shoot_flag--;
        }else{// is support and don't need shoot
            roleAry[env.RobotNumber] = this_robot_role;
        }
    }else{// is attacker
        if(fabs(Current_time-Begin_time)<=0.5){
           roleAry[env.RobotNumber] = Role_Halt;
        }else{
           roleAry[env.RobotNumber] = this_robot_role;
        }
    }

    int attacker_select_starter;// which role you are when 2sec, will continue on teamstrategy
    if(this_robot_role==Role_NewSupport){
        attacker_select_starter=2;
    }else if(this_robot_role==Role_Attack){
        attacker_select_starter=1;
    }
    if(env.isteamstrategy==1&&fabs(Current_time-Begin_time)>=2){//if teamstrategy open
        static int attacker_select = attacker_select_starter;
        static int this_attacker_select_counter=0;
        static int another_attacker_select_counter=0;
        static int two_robot_get_ball_counter=0;
        switch(attacker_select){
            case 0:// lost ball state
                printf("lost ball state\n");
                roleAry[env.RobotNumber]=Role_Attack;
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

                if(env.AnotherGoalDistance<distance_dr&&distance_dr<1.6){
                    roleAry[env.RobotNumber]=Role_NewSupport;
                }
            break;
            case 1:// this robot catch ball state
                printf("this robot catch ball\n");
                roleAry[env.RobotNumber]=Role_Attack;
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
                printf("another robot catch ball\n");
                roleAry[env.RobotNumber]=Role_NewSupport;
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
    }
}

void FIRA_teamStrategy_class::role_CornerKick(){
    printf("Corner Kick\n");
    Current_time = ros::Time::now().toSec();
    printf("Current_time-Begin_time=%f\n",fabs(Current_time-Begin_time));
    double distance_br=env.home[env.RobotNumber].ball.distance;//this robot
    double distance_dr=env.home[env.RobotNumber].goal.distance;
    double angle_br=env.home[env.RobotNumber].ball.angle;//this robot
    double angle_chase = Chase_Strategy[3];//16.5
    double distance_chase = Chase_Strategy[4];//0.4
    //depend on if this robot catch ball, use role flag decide role
    if((distance_br<=distance_chase)&&(fabs(angle_br)<=angle_chase)){//if catch ball
        if(role_flag==1){
            this_robot_role = Role_NewSupport;
            role_flag--;
        }
    }else{//not catch ball
        if(role_flag==1){
            this_robot_role = Role_Attack;
            role_flag--;
        }
    }
    if(this_robot_role == Role_NewSupport){
        if(shoot_flag==1){// is support and need shoot
            roleAry[env.RobotNumber] = Role_Kick;
            shoot_flag--;
        }else{// is support and don't need shoot
            roleAry[env.RobotNumber] = this_robot_role;
        }
    }else{// is attacker
        roleAry[env.RobotNumber] = this_robot_role;
    }

    int attacker_select_starter;// which role you are when 2sec, will continue on teamstrategy
    if(this_robot_role==Role_NewSupport){
        attacker_select_starter=2;
    }else if(this_robot_role==Role_Attack){
        attacker_select_starter=1;
    }
    if(env.isteamstrategy==1&&fabs(Current_time-Begin_time)>=2){//if teamstrategy open
        static int attacker_select = attacker_select_starter;
        static int this_attacker_select_counter=0;
        static int another_attacker_select_counter=0;
        static int two_robot_get_ball_counter=0;
        switch(attacker_select){
            case 0:// lost ball state
                printf("lost ball state\n");
                roleAry[env.RobotNumber]=Role_Attack;
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

                if(env.AnotherGoalDistance<distance_dr&&distance_dr<1.6){
                    roleAry[env.RobotNumber]=Role_NewSupport;
                }
            break;
            case 1:// this robot catch ball state
                printf("this robot catch ball\n");
                roleAry[env.RobotNumber]=Role_Attack;
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
                printf("another robot catch ball\n");
                roleAry[env.RobotNumber]=Role_NewSupport;
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
    }
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

