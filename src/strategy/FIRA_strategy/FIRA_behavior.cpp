#include "FIRA_behavior.h"
#include "math.h"
static double Begin_time = 0;
static double Current_time = 0;


FIRA_behavior_class::FIRA_behavior_class(){
   opponent = false;
}

//start---simulator---
void FIRA_behavior_class::setEnv(Environment iEnv){
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
//        To determine the robot state changes       //
//                                                   //
//###################################################//
void FIRA_behavior_class::StateInitAttack(int r_number){
        /// ========== Init Begin ==========
        double distance_br = env.home[r_number].ball.distance;
        double distance_dr = env.home[r_number].goal.distance;
        double angle_br = env.home[r_number].ball.angle;
        double angle_dr = env.home[r_number].goal.angle;
        double alpha = angle_dr-angle_br;
        /// ========== Init End ==========
        if(distance_br<=Chase_Strategy[4] && angle_br<=Chase_Strategy[3])
            state_attack = state_Attack;
        else
            state_attack = state_Chase;
}
void FIRA_behavior_class::StateChase(int r_number){
    /// ========== Init Begin ==========
    //    printf("Chase_state\n");
        double distance_br = env.home[r_number].ball.distance;
        double distance_dr = env.home[r_number].goal.distance;
        double angle_br = env.home[r_number].ball.angle;
        double angle_dr = env.home[r_number].goal.angle;
        double alpha = angle_dr-angle_br;
    /// ========== Init End ==========
        ////========state change to sidespeedup or not============
                /// ========== USE ARRAY TO SAVE DISTANCE AND CAUCULATE ==========
                int const  ary_length = 7;
                double temp;
                static double d_ary_dis[ary_length];
                double d_ary_orderdis[ary_length];
                static int counter = 0;
                static int bool_ary_full = false;
                static ros::Time start = ros::Time::now();
                ros::Time current = ros::Time::now();
                double start_time = (double)(start.sec+(double)start.nsec/1000000000);
                double current_time = (double)(current.sec+(double)current.nsec/1000000000);
                double const open_condition = Chase_Strategy[1];//0.15
                double const calculate_time = Chase_Strategy[2];//0.1
                if(current_time-start_time>calculate_time){
                    start.sec = current.sec;
                    start.nsec = current.nsec;
                    d_ary_dis[counter++] = distance_br;
                    if(counter==ary_length){
                        bool_ary_full = true;
                        counter = 0;
                    }
                    if(bool_ary_full==true){
                            for(int i = 0;i<ary_length;i++)
                                d_ary_orderdis[i] = d_ary_dis[i];
                            for(int i=0;i<ary_length;i++)
                                for(int j=0;j<ary_length-i-1;j++)
                                    if(d_ary_orderdis[j]>d_ary_orderdis[j+1]){
                                            temp = d_ary_orderdis[j];
                                            d_ary_orderdis[j] = d_ary_orderdis[j+1];
                                            d_ary_orderdis[j+1] = temp;
                                    }
                            if(d_ary_orderdis[ary_length-1]-d_ary_orderdis[0] > 0.05 &&
                               d_ary_orderdis[ary_length-1]-d_ary_orderdis[0] < open_condition &&
                               distance_br>=0.5){
                               counter = 0;
                               bool_ary_full = false;
                               for(int i = 0;i<ary_length;i++)
                                    d_ary_orderdis[i] = 0;
                            }
                    }
                }
                /// ========== USE LINKED LIST TO SAVE DISTANCE ==========

        ////========state change to Attack or not=================
          double angle_chase = Chase_Strategy[3];//16.5
          double distance_chase = Chase_Strategy[4];//0.4
        ////========== normalization angle to -180~180 ==========
          if(alpha>180)
              alpha-=360;
          else if(alpha<-180)
              alpha+=360;
        ////========== normalization end ==========
        static int counter_test = 0;
        if(distance_br<=distance_chase && fabs(angle_br)<=angle_chase){
            counter_test++;
            if(counter_test>1){
                state_attack=state_Attack;
                state_cornerkick=state_Attack;
                counter_test = 0;
                counter=0;          // when state leave from Chase   counter must to reset
            }
        }else counter_test = 0;
}
void FIRA_behavior_class::StateAttack(int r_number){
    /// ========== Init Begin ==========
    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double alpha = angle_dr-angle_br;
    /// ========== Init End ==========
    double const angle_attack = Attack_Strategy[0];//30.0
    double const distance_attack = Attack_Strategy[1];//1.0
    double const distance_chase = Attack_Strategy[2];//1.0
    ////========== normalization angle to -180~180 ==========
      if(alpha>180)
          alpha-=360;
      else if(alpha<-180)
          alpha+=360;
    ////========== normalization end ==========
    static int counter_test = 0;
    if(distance_br>distance_chase ||fabs(angle_br) > angle_attack){
        counter_test++;
        if(counter_test>15){
            state_attack=state_Chase;
            state_cornerkick=state_Chase;
            counter_test = 0;
        }
    }else counter_test = 0;
   if(distance_dr < distance_attack /*&& fabs(alpha)<=10*/ && distance_dr >= -100) {
       state_attack=state_ZoneAttack;
   }
}
void FIRA_behavior_class::StateType_UChase(int r_number){
    /// ========== Init Begin ==========
    //    printf("Chase_state\n");
        double distance_br = env.home[r_number].ball.distance;
        double distance_dr = env.home[r_number].goal.distance;
        double angle_br = env.home[r_number].ball.angle;
        double angle_dr = env.home[r_number].goal.angle;
        double alpha = angle_dr-angle_br;
    /// ========== Init End ==========
    ////========state change to Attack or not=================
      double angle_chase = Chase_Strategy[3];//16.5
      double distance_chase = Chase_Strategy[4];//0.4
    ////========== normalization angle to -180~180 ==========
      if(alpha>180)
          alpha-=360;
      else if(alpha<-180)
          alpha+=360;
    ////========== normalization end ==========
      static int counter_test = 0;
    if(distance_br<=distance_chase && fabs(angle_br)<=angle_chase){
        counter_test++;
        if(counter_test>1){
            state_attack=state_Attack;
            state_cornerkick=state_Attack;
            counter_test = 0;
        }
    }else counter_test = 0;
}
void FIRA_behavior_class::StateType_SAttack(int r_number){
//
}
void FIRA_behavior_class::StateSideSpeedUp(int r_number){
    /// ========== Init Begin ==========
    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double alpha = angle_dr-angle_br;
    /// ========== Init End ==========
    double angle_side=Side_Speed_Up[0];//10
    double const distance_side=Side_Speed_Up[1];//0.45
    ////========== normalization angle to -180~180 ==========
      if(alpha>180)
          alpha-=360;
      else if(alpha<-180)
          alpha+=360;
    ////========== normalization end ==========
         /// =========== CHANGE TO ATTACK ============
    if(fabs(alpha)<=angle_side && distance_br<=distance_side)
            state_attack=state_Attack;
         ///=========== CHANGE TO CHASE ===========
              ///========== USE TIMER ==========
    static ros::Time start = ros::Time::now();
    ros::Time current = ros::Time::now();
    double start_time = (double)(start.sec+(double)start.nsec/1000000000);
    double current_time = (double)(current.sec+(double)current.nsec/1000000000);
    double const int_calculate_time = 1.5;
    if(current_time-start_time > int_calculate_time){
        start.sec = current.sec;
        start.nsec = current.nsec;
        state_attack = state_Chase;
    }
}
void FIRA_behavior_class::StateZoneAttack(int r_number){
    /// ========== Init Begin ==========
    double distance_br = env.home[r_number].ball.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double alpha = angle_dr-angle_br;
    /// ========== Init End ==========
    double distance_zone =Zone_Attack[0];//0.6
    ////========== normalization angle to -180~180 ==========
      if(alpha>180)
          alpha-=360;
      else if(alpha<-180)
          alpha+=360;
    ////========== normalization end ==========
    if(distance_br > distance_zone){
            state_attack=state_Chase;
            state_cornerkick=state_Chase;
    }
}

void FIRA_behavior_class::StateCornerKick(int r_number){
    /// ========== Init Begin ==========
    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double alpha = angle_dr-angle_br;
    ////========== normalization angle to -180~180 ==========
      if(alpha>180)
          alpha-=360;
      else if(alpha<-180)
          alpha+=360;
    ////========== normalization end ==========
    /// ========== Init End ==========
     /// ========== param Begin ==========
      double angle_corner = Corner_Kick[0];//10
      double distance_corner = Corner_Kick[1];//0.45
     /// ========== param End ==========
    if(fabs(angle_dr)<angle_corner && distance_br<=distance_corner)
        state_cornerkick=state_Attack;
    if(distance_br > 1)
        EscapeCornerKick[r_number] = true;
}
//###################################################//
//                                                   //
//            Read the character array               //
//                                                   //
//###################################################//
void FIRA_behavior_class::readroleAry(int robotIndex,int role){
//        printf("roleplay=%d\n",role);
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
            case Role_PenaltyKick:
                behavior_PenaltyKick(robotIndex);
                break;
            case Role_ThrowIn:
                behavior_ThrowIn(robotIndex);
                break;
            case Role_CornerKick:
                if(EscapeCornerKick[robotIndex]){
                    behavior_Attack(robotIndex);
                    break;
                }else{
                    behavior_CornerKick(robotIndex);
                    break;
                }
            case Role_Test1:
                   behavior_Test1(robotIndex);
                    break;
            case Role_Test2:
                   behavior_Test2(robotIndex);
                   break;
            case Role_Test3:
                   behavior_Test3(robotIndex);
                   break;
            case Role_NewSupport:
                   behavior_NewSupport(robotIndex);
                   break;
            case Role_Kick:
                   behavior_Kick(robotIndex);
                   break;
            }
}
//###################################################//
//                                                   //
//             return to strategy.cpp                //
//                                                   //
//###################################################//
int* FIRA_behavior_class::getactionAry(){
    return actionAry;
}
//###################################################//
//                                                   //
//      Select the appropriate path planning         //
//                                                   //
//###################################################//
void FIRA_behavior_class::behavior_Goalkeeper(int robotIndex){
    actionAry[robotIndex] = action_Goalkeeper;
}

void FIRA_behavior_class::behavior_Attack(int robotIndex){
    
    static bool run_onetime = 0;
        if(run_onetime == 0)
        run_onetime = 1;

        int chaseCase = Strategy_Selection[0];
        int SchaseCase = Strategy_Selection[1];
        int attackCase = Strategy_Selection[2];
        int SattackCase = Strategy_Selection[3];
        int DattackCase = Strategy_Selection[4];
        int ShootCase = Strategy_Selection[5];

        double rushDistance = TypeS_Attack[2];

        switch(state_attack){
            case state_Init:
                state_attack = state_Init;
                StateInitAttack(robotIndex);
                ROS_INFO("init state\n");
                break;
            case state_Chase:
                if(chaseCase){
                    actionAry[robotIndex] = action_Chase;
                    ROS_INFO("chase state\n");
                }else if(SchaseCase){
                    actionAry[robotIndex] = action_Straight_Chase;
                    ROS_INFO("Straight chase state\n");
                }
                StateChase(robotIndex);
                break;
            case state_Attack:
                if(attackCase){
                    actionAry[robotIndex] = action_Attack;
                    ROS_INFO("Attack state\n");
                }else if(SattackCase){
                    if(env.home[robotIndex].goal.distance > rushDistance){ // if it is close enough to rush
                        actionAry[robotIndex] = action_typeS_attack;
                        ROS_INFO("S attack\n");
                    }else{
                        actionAry[robotIndex] = action_Attack;
                        ROS_INFO("Attack state\n");
                    }
                }else if(DattackCase){
                    actionAry[robotIndex] = action_Dorsad_Attack;
                    ROS_INFO("D attack\n");
                }else if(ShootCase){
                    actionAry[robotIndex] = action_Shoot_Attack;
                    ROS_INFO("Shoot attack\n");
                }
                StateAttack(robotIndex);
                break;
            case state_SideSpeedUp:
                actionAry[robotIndex] = action_SideSpeedUp;
                StateSideSpeedUp(robotIndex);
                ROS_INFO("SideSpeedUp state\n");
                break;
            case state_ZoneAttack:
                actionAry[robotIndex] = action_Zone_Attack;
//                state_attack[robotIndex] = state_Init;
                StateZoneAttack(robotIndex);
                ROS_INFO("Zone_Attack\n");
                break;
            }
}
void FIRA_behavior_class::behavior_Support(int robotIndex){
    int r_number=robotIndex;
    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double test_angle = angle_dr - op_angle_dr;
    static int left_count=0;
    static int right_count=0;
    static int left_right = 0;
    if(test_angle<0){
        test_angle = test_angle +360;
    }
    if(test_angle>=180){//right
        right_count++;
    }else{//left
        left_count++;
    }
    if(left_count>=10){
        printf("left side\n");
        printf("test_angle = %f\n",test_angle);
        right_count=0;
        left_count=0;
        left_right=1;
    }else if(right_count>=10){
        printf("right side\n");
        printf("test_angle = %f\n",test_angle);
        right_count=0;
        left_count=0;
        left_right=2;
    }
        int order= 1;

        if(((distance_br>=9.99)||(distance_dr>=9.99))||(op_distance_dr>=9.99)){
            printf("action_Halt br:%f, dr:%f op_dr:%f\n",distance_br,distance_dr,op_distance_dr);
            actionAry[robotIndex] = action_Halt;
        }else if(op_distance_dr<1.6){// in limit area
            actionAry[robotIndex] = action_LeaveLimitArea;
            printf("action_LeaveLimitArea\n");
        }else if(distance_br<0.6||(angle_br>120||angle_br<-120)){// too close ball leave ball
            actionAry[robotIndex] = action_LeaveBall;
            printf("action_LeaveBall\n");
        }else if(op_distance_dr>4){// too far from defend gate
            actionAry[robotIndex] = action_MovetoOpGoal;
            printf("action_MovetoYellowGate\n");
        }else if(distance_br>=0.6){//all good above, left right defend ball state
            switch(env.R1OrderR2){// order = 1, go to right. order = 2 , go to left
                case 1:
                    //actionAry[robotIndex] = action_MovetoGoalEdge2;//right

                    if(left_right==1){// left side, go to right
                        actionAry[robotIndex] = action_MovetoGoalEdge2;
                        printf("left side, go to right\n");
                    }else if(left_right==2){// right side go to right
                        actionAry[robotIndex] = action_Support_Test3;
                        printf("right side, go to right\n");
                    }
                break;
                case 2:
                    if(left_right==1){// left side go to left
                        actionAry[robotIndex] = action_Support_Test1;//go to left
                        printf("left side, go to left\n");
                    }else if(left_right==2){// right side go to left
                        actionAry[robotIndex] = action_MovetoGoalEdge1;//op left
                        printf("right side, go to left\n");
                    }

                break;
//                case 3:
//                    actionAry[robotIndex] = action_MovetoGoalEdge2;
//                break;
            }

            printf("left right defend ball state\n");
        }else{// impossible to be here
            printf("action_Halt\n");
            actionAry[robotIndex] = action_Halt;
        }
    //actionAry[robotIndex] = action_Support;
}
void FIRA_behavior_class::behavior_Halt(int robotIndex){
    ///===================reset ===================
    EscapeCornerKick[robotIndex] = false;
    actionAry[robotIndex] = action_Halt;
    state_cornerkick = state_CornerKick;
    state_attack = state_Init;
    Begin_time = ros::Time::now().toSec();
    Current_time = ros::Time::now().toSec();
//    printf("Begin_time=%f\n",Begin_time);
//    printf("Current_time=%f\n",Current_time);
    ///=========================================
}
void FIRA_behavior_class::behavior_AvoidBarrier(int robotIndex){
    actionAry[robotIndex] = action_AvoidBarrier;
}
void FIRA_behavior_class::behavior_PenaltyKick(int robotIndex){
    actionAry[robotIndex] = action_PenaltyKick;
}
void FIRA_behavior_class::behavior_ThrowIn(int robotIndex){
    actionAry[robotIndex] = action_ThrowIn;
}
void FIRA_behavior_class::behavior_CornerKick(int robotIndex){
        state_cornerkick = state_CornerKick;
        if(!EscapeCornerKick[robotIndex]){
            switch(state_cornerkick){
            case state_CornerKick:
                actionAry[robotIndex] = action_CornerKick;
                StateCornerKick(robotIndex);
                break;
            case state_Attack:
                actionAry[robotIndex] = action_Attack;
                break;
            }
        }
}
void FIRA_behavior_class::behavior_Test1(int robotIndex){

    int r_number=robotIndex;
    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
//    printf("final_angle=%f\n",env.Support_Obstacle_angle);
//    printf("final_distance=%f\n",env.Support_Obstacle_distance);
    if(op_distance_dr<1.5){// in limit area
        actionAry[robotIndex] = action_LeaveLimitArea;
        printf("action_LeaveLimitArea\n");
    }else if((angle_br>120||angle_br<-120)||distance_br<1){
        actionAry[robotIndex] = action_LeaveBall;
        printf("action_Chase\n");
    }else if(op_distance_dr>3.5){// too far from defend gate
        actionAry[robotIndex] = action_MovetoOpGoal;
        printf("action_MovetoYellowGate\n");
    }else{//all good above, left right defend ball state
        actionAry[robotIndex] = action_LeftRightMove;
        printf("left right defend ball state\n");
    }
    //printf("test1\n");



}
void FIRA_behavior_class::behavior_Test2(int robotIndex){
    double distance_br = env.home[robotIndex].ball.distance;
    double distance_dr = env.home[robotIndex].goal.distance;
    double op_distance_dr = env.home[robotIndex].op_goal.distance;
    double angle_br = env.home[robotIndex].ball.angle;
    double angle_dr = env.home[robotIndex].goal.angle;
    double op_angle_dr = env.home[robotIndex].op_goal.angle;
    double v_rotation=0;
    //printf("distance_br=%f\n",distance_br);


    if(mTeam == Team_Yellow){
        if(op_distance_dr<1.5){// in limit area
            actionAry[robotIndex] = action_LeaveLimitArea;
            printf("action_LeaveLimitArea\n");
        }else if(angle_br>120||angle_br<-120){
            actionAry[robotIndex] = action_Chase;
            printf("action_Chase\n");
        }else if(op_distance_dr>3.5){// too far from defend gate
            actionAry[robotIndex] = action_MovetoYellowGate;
            printf("action_MovetoYellowGate\n");
        }else{//all good above, left right defend ball state
            actionAry[robotIndex] = action_Support_LostInternet;
            printf("left right defend ball state\n");
        }
    }else if(mTeam == Team_Blue){
        if(op_distance_dr<1.5){// in limit area
            actionAry[robotIndex] = action_LeaveLimitArea;
            printf("action_LeaveLimitArea\n");
        }else if(angle_br>120||angle_br<-120){
            actionAry[robotIndex] = action_Chase;
            printf("action_Chase\n");
        }else if(op_distance_dr>3.5){// too far from defend gate
            actionAry[robotIndex] = action_MovetoBlueGate;
            printf("action_MovetoBlueGate\n");
        }else{//all good above, left right defend ball state
            actionAry[robotIndex] = action_Support_LostInternet;
            printf("left right defend ball state\n");
        }
    }

    if(((distance_br>=9.99)||(distance_dr>=9.99))||(op_distance_dr>=9.99)){
        printf("action_Halt br:%f, dr:%f op_dr:%f\n",distance_br,distance_dr,op_distance_dr);
        actionAry[robotIndex] = action_Halt;
    }
}
void FIRA_behavior_class::behavior_Test3(int robotIndex){

    int r_number=robotIndex;
    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double test_angle = angle_dr - op_angle_dr;
    static int left_count=0;
    static int right_count=0;
    static int left_right = 0;
    if(test_angle<0){
        test_angle = test_angle +360;
    }
    if(test_angle>=180){//right
        right_count++;
    }else{//left
        left_count++;
    }
    if(left_count>=10){
        printf("left side\n");
        printf("test_angle = %f\n",test_angle);
        right_count=0;
        left_count=0;
        left_right=1;
    }else if(right_count>=10){
        printf("right side\n");
        printf("test_angle = %f\n",test_angle);
        right_count=0;
        left_count=0;
        left_right=2;
    }
        int order= 1;

        if(((distance_br>=9.99)||(distance_dr>=9.99))||(op_distance_dr>=9.99)){
            printf("action_Halt br:%f, dr:%f op_dr:%f\n",distance_br,distance_dr,op_distance_dr);
            actionAry[robotIndex] = action_Halt;
        }else if(op_distance_dr<1.6){// in limit area
            actionAry[robotIndex] = action_LeaveLimitArea;
            printf("action_LeaveLimitArea\n");
        }else if(distance_br<0.6||(angle_br>120||angle_br<-120)){// too close ball leave ball
            actionAry[robotIndex] = action_LeaveBall;
            printf("action_LeaveBall\n");
        }else if(op_distance_dr>4){// too far from defend gate
            actionAry[robotIndex] = action_MovetoOpGoal;
            printf("action_MovetoYellowGate\n");
        }else if(distance_br>=0.6){//all good above, left right defend ball state
            switch(env.R1OrderR2){// order = 1, go to right. order = 2 , go to left
                case 1:
                    //actionAry[robotIndex] = action_MovetoGoalEdge2;//right

                    if(left_right==1){// left side, go to right
                        actionAry[robotIndex] = action_MovetoGoalEdge2;
                        printf("left side, go to right\n");
                    }else if(left_right==2){// right side go to right
                        actionAry[robotIndex] = action_Support_Test3;
                        printf("right side, go to right\n");
                    }
                break;
                case 2:
                    if(left_right==1){// left side go to left
                        actionAry[robotIndex] = action_Support_Test1;//go to left
                        printf("left side, go to left\n");
                    }else if(left_right==2){// right side go to left
                        actionAry[robotIndex] = action_MovetoGoalEdge1;//op left
                        printf("right side, go to left\n");
                    }

                break;
//                case 3:
//                    actionAry[robotIndex] = action_MovetoGoalEdge2;
//                break;
            }

            printf("left right defend ball state\n");
        }else{// impossible to be here
            printf("action_Halt\n");
            actionAry[robotIndex] = action_Halt;
        }
        printf("env.R1OrderR2=%d\n",env.AnotherRobotNumber);
        printf("env.R1OrderR2=%f\n",env.AnotherBallDistance);
        printf("env.R1OrderR2=%d\n",env.AnotherGetBall);
        printf("env.R1OrderR2=%f\n",env.AnotherGoalDistance);
        printf("env.R1OrderR2=%d\n",env.R1OrderR2);

}
void FIRA_behavior_class::behavior_NewSupport(int robotIndex){
    int r_number=robotIndex;
    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
//    printf("game.state=%d\n",env.gameState);
//    printf("Begin_time=%f\n",Begin_time);
//    printf("Current_time=%f\n",Current_time);
    Current_time = ros::Time::now().toSec();

    switch(Support_Strategy[0]){
        case 1:     //AutoCase
            printf("AutoCase\n");
            if(op_distance_dr<1.5){// in limit area
                actionAry[robotIndex] = action_LeaveLimitArea;
                printf("action_LeaveLimitArea\n");
            }else if((angle_br>120||angle_br<-120)||distance_br<1){
                actionAry[robotIndex] = action_LeaveBall;
                printf("action_Chase\n");
            }else if(op_distance_dr>3.5){// too far from defend gate
                actionAry[robotIndex] = action_MovetoOpGoal;
                printf("action_MovetoYellowGate\n");
            }else{//all good above, left right defend ball state
                actionAry[robotIndex] = action_LeftRightMove;
                printf("left right defend ball state\n");
            }
            break;
        case 2:     //BlockCase
            printf("BlockCase\n");
            if(fabs(Current_time-Begin_time)>=4){
                if(op_distance_dr<1.5){// in limit area
                    actionAry[robotIndex] = action_LeaveLimitArea;
                    printf("action_LeaveLimitArea\n");
                }else if((angle_br>120||angle_br<-120)||distance_br<1){
                    actionAry[robotIndex] = action_LeaveBall;
                    printf("action_Chase\n");
                }else if(op_distance_dr>3.5){// too far from defend gate
                    actionAry[robotIndex] = action_MovetoOpGoal;
                    printf("action_MovetoYellowGate\n");
                }else{//all good above, left right defend ball state
                    actionAry[robotIndex] = action_LeftRightMove;
                    printf("left right defend ball state\n");
                }
            }else{
                actionAry[robotIndex] = action_Block;
            }
            break;
        case 3:     //HideCase
            printf("HideCase\n");
            break;
        case 4:
            printf("teamstrategy leftright move\n");
            break;
        case 5:
            printf("Support BE ATTACK!\n");
            behavior_Attack(robotIndex);
            break;
        default:
            printf("default\n");
            if(op_distance_dr<1.5){// in limit area
                actionAry[robotIndex] = action_LeaveLimitArea;
                printf("action_LeaveLimitArea\n");
            }else if((angle_br>120||angle_br<-120)||distance_br<1){
                actionAry[robotIndex] = action_LeaveBall;
                printf("action_Chase\n");
            }else if(op_distance_dr>3.5){// too far from defend gate
                actionAry[robotIndex] = action_MovetoOpGoal;
                printf("action_MovetoYellowGate\n");
            }else{//all good above, left right defend ball state
                actionAry[robotIndex] = action_LeftRightMove;
                printf("left right defend ball state\n");
            }
            break;
    }
    //printf("XXXXXXCurrent_time-Begin_time=%f\n",fabs(Current_time-Begin_time));


}
void FIRA_behavior_class::behavior_Kick(int robotIndex){

    actionAry[robotIndex] = action_Kick;
}
//###################################################//
//                                                   //
//                 load parameter                    //
//                                                   //
//###################################################//
void FIRA_behavior_class::loadParam(ros::NodeHandle *n){
    if(n->getParam("/FIRA_Behavior/Attack_Strategy", Attack_Strategy)){
//        for(int i=0;i<3;i++)
//            std::cout<< "param Attack_Strategy["<< i << "]=" << Attack_Strategy[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA_Behavior/Chase_Strategy", Chase_Strategy)){
//        for(int i=0;i<5;i++)
//            std::cout<< "param Chase_Strategy["<< i << "]=" << Chase_Strategy[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA_Behavior/Corner_Kick", Corner_Kick)){
//        for(int i=0;i<2;i++)
//            std::cout<< "param Corner_Kick["<< i << "]=" << Corner_Kick[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA_Behavior/Side_Speed_UP", Side_Speed_Up)){
//        for(int i=0;i<2;i++)
//            std::cout<< "param Side_Speed_UP["<< i << "]=" << Side_Speed_UP[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA_Behavior/TypeS_Attack", TypeS_Attack)){
//        for(int i=0;i<3;i++)
//            std::cout<< "param TypeS_Attack["<< i << "]=" << TypeS_Attack[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA_Behavior/TypeU_Chase", TypeU_Chase)){
//        for(int i=0;i<2;i++)
//            std::cout<< "param TypeU_Chase["<< i << "]=" << TypeU_Chase[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA_Behavior/Zone_Attack", Zone_Attack)){
//        for(int i=0;i<1;i++)
//            std::cout<< "param Zone_Attack["<< i << "]=" << Zone_Attack[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/StrategySelection", Strategy_Selection)){

    }
    if(n->getParam("/FIRA_Behavior/Support_Strategy", Support_Strategy)){
//        for(int i=0;i<5;i++)
//            std::cout<< "param Support_Strategy["<< i << "]=" << Support_Strategy[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
}
