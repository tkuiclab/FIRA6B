#include "FIRA_behavior.h"
#include "math.h"



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
        if(distance_br>Chase_Strategy[4])
            state_attack = state_Chase;
        else
            state_attack = state_Attack;
}

void FIRA_behavior_class::StateChase(int r_number){

        double distance_br = env.home[r_number].ball.distance;
        double distance_dr = env.home[r_number].goal.distance;
        double angle_br = env.home[r_number].ball.angle;
        double angle_dr = env.home[r_number].goal.angle;
        double alpha = angle_dr-angle_br;

        double angle_chase = Chase_Strategy[3];//16.5
        double distance_chase = Chase_Strategy[4];//0.4

        if(alpha>180)
            alpha-=360;
        else if(alpha<-180)
            alpha+=360;

        int const  ary_length = 3;
        static double dis_ary[ary_length] = {0};
        static int counter = 0;
        static int bool_ary_full = false;

        if(distance_br<=distance_chase && fabs(angle_br)<=angle_chase){
                state_attack=state_Attack;
                state_cornerkick=state_Attack;
                //reset sidespeedup_checking_data
                counter = 0;
                bool_ary_full = false;
                for(int reset_ary = 0; reset_ary < ary_length; reset_ary++){
                    dis_ary[reset_ary] = 0;   }
        }


        static ros::Time start = ros::Time::now();
        ros::Time current = ros::Time::now();
        double start_time = (double)(start.sec+(double)start.nsec/1000000000);
        double current_time = (double)(current.sec+(double)current.nsec/1000000000);
        double const open_condition = Chase_Strategy[1];//0.05      //2017_0406
        double const calculate_time = Chase_Strategy[2];//0.08      //2017_0406


            if(current_time-start_time>calculate_time){
                start.sec = current.sec;
                start.nsec = current.nsec;
                if( bool_ary_full == true){
                    double average_sum_dis_ary = 0;
                    for(int i=0; i<ary_length; i++){
                        average_sum_dis_ary += dis_ary[i];
                    }
                    average_sum_dis_ary /= ary_length;
                    double check_for_sidespeedup = average_sum_dis_ary - distance_br;

                    if( open_condition > fabs(check_for_sidespeedup)){
                        state_attack=state_SideSpeedUp;
                        //reset sidespeedup_checking_data
                        counter = 0;
                        bool_ary_full = false;
                        for(int reset_ary = 0; reset_ary < ary_length; reset_ary++){
                            dis_ary[reset_ary] = 0;   }
                    }else{  dis_ary[counter] =0; }
                }
                dis_ary[counter++] += distance_br;
                if(counter==ary_length){
                    counter = 0;
                    bool_ary_full = true;
                }
            }
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
    double angle_side=Side_Speed_UP[0];//10
    double const distance_side=Side_Speed_UP[1];//0.45
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

        int chaseCase = StrategySelection[0];
        int SchaseCase = StrategySelection[1];

        double angle_stopsidespeedup =  5;    //Side_Speed_UP[2] ;
        if(chaseCase){
            if( distance_br <= 0.5 ){
                if( fabs(alpha) < angle_stopsidespeedup ){
                    state_attack=state_Chase;  }
            }
        }/*else if(SchaseCase){
            //直線追球在追到球前不須減速
        }*/

        if(current_time-start_time > int_calculate_time){   //1.5sec reset
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
//        To determine the robot state changes       //
//        for GoalKeeper
//                                                   //
//###################################################//
void FIRA_behavior_class::StateGoalKeeperInit(int r_number){
    printf("init\n");
    // 暫時無條件進入
    state_GoalKeeper = state_GoalKeeper_blocking;

//        double ball_distance = env.home[r_number].ball.distance;
//        double opgoal_distance = env.home[r_number].op_goal.distance;
//        double opgoal_angle = env.home[r_number].op_goal.angle;
//        double opgoal_edge_dis = env.home[r_number].opgoal_edge.distance;
//        double opgoal_edge_angle1 = env.home[r_number].opgoal_edge.angle_1;
//        double opgoal_edge_angle2 = env.home[r_number].opgoal_edge.angle_2;
//        if(opgoal_distance < 0.7 && fabs(opgoal_angle) > 175 ){
//            state_GoalKeeper = state_GoalKeeper_waiting;
//        }

}

void FIRA_behavior_class::StateGoalKeeperWaiting(int r_number){

//        double ball_distance = env.home[r_number].ball.distance;
//        if( ball_distance < 3 ){
//            state_GoalKeeper = state_GoalKeeper_blocking;
//        }

}


void FIRA_behavior_class::StateGoalKeeperBlocking(int r_number){
    printf("block");
        double ball_dis = env.home[r_number].ball.distance;
        double opgoal_dis = env.home[r_number].op_goal.distance;

        if( ball_dis < 1.5 && opgoal_dis < 0.95){
            state_GoalKeeper = state_GoalKeeper_catching;
        }else if(ball_dis < 1 && opgoal_dis < 1.1){
            state_GoalKeeper = state_GoalKeeper_catching;
        }
}

void FIRA_behavior_class::StateGoalKeeperCatching(int r_number){
    printf("push");
    double ball_dis = env.home[r_number].ball.distance;
    double opgoal_dis = env.home[r_number].op_goal.distance;
    if( opgoal_dis > 1.7 || ball_dis > 2){
        state_GoalKeeper = state_GoalKeeper_blocking;
    }
}



//###################################################//
//                                                   //
//            Read the character array               //
//                                                   //
//###################################################//
void FIRA_behavior_class::readroleAry(int robotIndex,int role){
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
            case Role_CornerKick:
                if(EscapeCornerKick[robotIndex]){
                    behavior_Attack(robotIndex);
                    break;
                }else{
                    behavior_CornerKick(robotIndex);
                    break;
                }
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
    switch(state_GoalKeeper){
        case state_GoalKeeper_init:
            state_GoalKeeper = state_GoalKeeper_init;
            StateGoalKeeperInit(robotIndex);
            //ROS_INFO("GoalKeeper init state\n");
            actionAry[robotIndex] = action_Goalkeeper_init;
            break;

        case state_GoalKeeper_waiting:
            StateGoalKeeperWaiting(robotIndex);
            //ROS_INFO("GoalKeeper waiting\n");
            actionAry[robotIndex] = action_Goalkeeper_waiting;
            break;

        case state_GoalKeeper_blocking:
            StateGoalKeeperBlocking(robotIndex);
//            ROS_INFO("GoalKeeper blocking\n");
            actionAry[robotIndex] = action_Goalkeeper_blocking;
        break;

        case state_GoalKeeper_catching:
            StateGoalKeeperCatching(robotIndex);
//            ROS_INFO("GoalKeeper catching\n");
            actionAry[robotIndex] = action_Goalkeeper_catching;
        break;
    }

}

void FIRA_behavior_class::behavior_Attack(int robotIndex){
    
    static bool run_onetime = 0;
        if(run_onetime == 0)
        run_onetime = 1;

        int chaseCase = StrategySelection[0];
        int SchaseCase = StrategySelection[1];
        int attackCase = StrategySelection[2];
        int SattackCase = StrategySelection[3];
        int DattackCase = StrategySelection[4];
        int ShootCase = StrategySelection[5];

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
                StateZoneAttack(robotIndex);
                ROS_INFO("Zone_Attack\n");
                break;
            }
}
void FIRA_behavior_class::behavior_Support(int robotIndex){
    actionAry[robotIndex] = action_Support;
}
void FIRA_behavior_class::behavior_Halt(int robotIndex){
    ///===================reset ===================
    EscapeCornerKick[robotIndex] = false;
    actionAry[robotIndex] = action_Halt;
    state_cornerkick = state_CornerKick;
    state_attack = state_Init;
    ///=========================================
}
void FIRA_behavior_class::behavior_AvoidBarrier(int robotIndex){
    actionAry[robotIndex] = action_AvoidBarrier;
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
    if(n->getParam("/FIRA_Behavior/Side_Speed_UP", Side_Speed_UP)){
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
    if(n->getParam("/StrategySelection", StrategySelection)){

    }
}
