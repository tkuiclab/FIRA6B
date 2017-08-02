#include "FIRA_pathplan.h"
#include "math.h"
#include "time.h"
#include <iostream>
using namespace std;
static double Begin_time = 0;
static double Current_time = 0;
static bool loopEnd;

//=========Environment init=============//
FIRA_pathplan_class::FIRA_pathplan_class(){
   opponent = false;
}
void FIRA_pathplan_class::setEnv(Environment iEnv){
   env = iEnv;
   if(opponent){
       for(int i = 0;i < PLAYERS_PER_SIDE;i++){
           env.home[i] = iEnv.opponent[i];
           env.opponent[i] = iEnv.home[i];
       }
   }
}
//=========Environment end=============//
//###################################################//
//                                                   //
//                Old path planning                  //
//                                                   //
//###################################################//
void FIRA_pathplan_class::strategy_head2ball(int i)
{
   Vector3D ball = env.currentBall.pos;
   Vector3D robot = env.home[i].pos;
   double robot_rotation = env.home[i].rotation;
   double x = ball.x - robot.x;
   if (x==0) x= very_small;
   double y = ball.y - robot.y;
   double a = atan2(y,x)*rad2deg;
   double angle = a - robot_rotation;

   if(angle<-180)angle = angle+360;
   if(angle>180)angle = angle-360;

   env.home[i].v_yaw = angle*5;
}

void FIRA_pathplan_class::strategy_dst(double target_x,double target_y){
   double v_x = target_x - env.home[0].pos.x;
   double v_y = target_y - env.home[0].pos.y;

   env.home[0].v_x = v_x;
   env.home[0].v_y = v_y;
}

void FIRA_pathplan_class::strategy_dst_head2ball(double target_x,double target_y){

   Vector3D ball = env.currentBall.pos;
   Vector3D robot = env.home[2].pos;

   double x = target_x - robot.x;
   double y = target_y - robot.y;
   double angle;
   double robot_rotation = env.home[2].rotation;
   double tar_dis = sqrt(x*x + y*y);
   double tar_ang = atan2(y,x)*rad2deg;
   double v,w;

   double x_ball = ball.x - robot.x;
   if (x_ball==0) x_ball = very_small;
   double y_ball = ball.y - robot.y;

   double a = atan2(y_ball,x_ball) * rad2deg;
   angle = a - robot_rotation ;

   if(angle<-180) angle += 360;
   if(angle>=180) angle -= 360;

   //---speed planning---
   if(tar_dis < 0.1){
       v=0;
       if(fabs(angle) < 5) w=0;
       else w = angle/fabs(angle) * 200 * (1-exp(-1*fabs(angle)/10))+10;  //angular velocity
   }else{
       v = 250 * (1-exp(-1*tar_dis/10)) +10; //velocity
       w = 0;
   }
       env.home[2].v_x = x;
       env.home[2].v_y = y;
       env.home[2].v_yaw = w;//angle;
}
//###################################################//
//                                                   //
//          receive actionAry to execute             //
//                                                   //
//###################################################//
void FIRA_pathplan_class::personalStrategy(int robotIndex,int action){
        switch(action){
            case action_Goalkeeper:
                strategy_Goalkeeper(robotIndex);
                break;
            case action_Attack:
                strategy_Attack(robotIndex);
                break;
            case action_typeS_attack:
                strategy_typeS_Attack(robotIndex);
                break;
            case action_Dorsad_Attack:
                strategy_Dorsad_Attack(robotIndex);
                break;
            case action_Shoot_Attack:
                strategy_Shoot_Attack(robotIndex);
                break;
            case action_Chase:
                strategy_Chase(robotIndex);
                break;
            case action_Straight_Chase:
                strategy_Straight_Chase(robotIndex);
                break;
            case action_typeU_Attack:
                strategy_typeU_Attack(robotIndex);
                break;
            case action_Support:
                strategy_Support(robotIndex);
                break;
            case action_AvoidBarrier:
                strategy_AvoidBarrier(robotIndex);
                break;
            case Role_Halt:
                strategy_Halt(robotIndex);
                break;
            case action_SideSpeedUp:
                strategy_SideSpeedUp(robotIndex);
                break;
            case action_PenaltyKick:;
                strategy_PenaltyKick(robotIndex);
                break;
            case action_ThrowIn:
                strategy_ThrowIn(robotIndex);
                break;
            case action_CornerKick:
                strategy_CornerKick(robotIndex);
                break;
            case  action_Zone_Attack:
                strategy_Zone_Attack(robotIndex);
                break;
            case  action_Support_CatchBallState:
                strategy_Support_CatchBallState(robotIndex);
                break;
            case  action_Support_LostBallState:
                strategy_Support_LostBallState(robotIndex);
                break;
            case  action_Support_Test1:
                strategy_Support_Test1(robotIndex);
                break;
            case  action_Support_Test2:
                strategy_Support_Test2(robotIndex);
                break;
            case  action_Support_Test3:
                strategy_Support_Test3(robotIndex);
                break;
            case  action_Support_Positioning:
                strategy_Support_Positioning(robotIndex);
                break;
            case  action_MovetoBlueGate:
                strategy_MovetoBlueGate(robotIndex);
                break;
            case  action_MovetoYellowGate:
                strategy_MovetoYellowGate(robotIndex);
                break;
            case  action_LeaveBall:
                strategy_LeaveBall(robotIndex);
                break;
            case  action_LeaveLimitArea:
                strategy_LeaveLimitArea(robotIndex);
                break;
            case  action_LeftRightMove:
                strategy_LeftRightMove(robotIndex);
                break;
            case  action_inv_LeftRightMove:
                strategy_invLeftRightMove(robotIndex);
                break;
            case  action_Support_LostInternet:
                strategy_Support_LostInternet(robotIndex);
                break;
            case  action_MovetoGoal:
                strategy_MovetoGoal(robotIndex);
                break;
            case  action_MovetoOpGoal:
                strategy_MovetoOpGoal(robotIndex);
                break;
            case  action_MovetoGoalEdge1:
                strategy_MovetoGoalEdge1(robotIndex);
                break;
            case  action_MovetoGoalEdge2:
                strategy_MovetoGoalEdge2(robotIndex);
                break;
            case  action_MovetoOpGoalEdge1:
                strategy_MovetoOpGoalEdge1(robotIndex);
                break;
            case  action_MovetoOpGoalEdge2:
                strategy_MovetoOpGoalEdge2(robotIndex);
                break;
            case  action_Stop:
                strategy_Stop(robotIndex);
                break;
            case  action_Block:
                strategy_Block(robotIndex);
                break;
            case  action_Kick:
                strategy_Kick(robotIndex);
                break;
            case  action_FreeKick:
                strategy_FreeKick(robotIndex);
                break;
            case action_Escape_Attack:
                strategy_Escape_Attack(robotIndex);
                break;
        }
}
//###################################################//
//                                                   //
//                New path planning                  //
//                                                   //
//###################################################//
void FIRA_pathplan_class::strategy_Goalkeeper(int Robot_index){

}

void FIRA_pathplan_class::strategy_Chase(int r_number){
    double ball_dis = env.home[r_number].ball.distance;
    double ball_angle = /*((fabs(ball_angle)>=0.00001)?0.00001:*/env.home[r_number].ball.angle;
    if(fabs(ball_angle)<=0.00001)ball_angle=0.00001;
    double c_ball_dis = sqrt((ball_dis*ball_dis)+(half_robot*half_robot) - (2*ball_dis*half_robot*cos(ball_angle*deg2rad)));
    int sign=1;
    if(ball_angle<0)sign=(-1);
    double c_ball_angle = sign*(180- rad2deg*acos(((c_ball_dis*c_ball_dis)+(half_robot*half_robot)-(ball_dis*ball_dis))/(2*c_ball_dis*half_robot)));
    if(ball_angle==0.00001)c_ball_angle=ball_angle;
    double goal_angle = env.home[r_number].goal.angle;
    double ball_x = -c_ball_dis*sin(c_ball_angle*deg2rad);
    double ball_y = c_ball_dis*cos(c_ball_angle*deg2rad);
    double alpha = goal_angle - c_ball_angle;
    double beta = atan2( beta_const , c_ball_dis) * rad2deg;

    Vector2d vectorbr(ball_x, ball_y);
    if(beta < alpha){
        alpha = beta;
    }else if(alpha < -beta){
        alpha = -beta;
    }
    double rotAngle = -alpha;
    Rotation2Dd rot( rotAngle * deg2rad);        // Rotate Matrix
    Vector2d vectornt = rot * vectorbr;
    env.home[r_number].v_x =vectornt(0);
    env.home[r_number].v_y =vectornt(1);
    env.home[r_number].v_yaw = goal_angle;
    shoot = 0;
}

void FIRA_pathplan_class::strategy_Straight_Chase(int robotIndex){
    int r_number = robotIndex;

    double distance_br = env.home[r_number].ball.distance;
    double angle_br = env.home[r_number].ball.angle;

    if(fabs(angle_br) <= 0.00001) angle_br=0.00001;
    double c_distance_br = sqrt((distance_br * distance_br)+(half_robot*half_robot) - (2*distance_br * half_robot*cos(angle_br*deg2rad)));
    int sign=1;
    if(angle_br < 0) sign=(-1);
    double c_angle_br = sign*(180- rad2deg*acos(((c_distance_br * c_distance_br)+(half_robot*half_robot)-(distance_br * distance_br))/(2*c_distance_br * half_robot)));
    if(angle_br==0.00001) c_angle_br = angle_br;

    double vectorbr_x = -c_distance_br * sin(c_angle_br * deg2rad);
    double vectorbr_y = c_distance_br * cos(c_angle_br * deg2rad);

    env.home[r_number].v_x = vectorbr_x;
    env.home[r_number].v_y = vectorbr_y;
    env.home[r_number].v_yaw = angle_br;
    shoot = 0;
}

void FIRA_pathplan_class::strategy_KO5_Chase(int r_number){

    double ball_dis = env.home[r_number].ball.distance;
    double ball_angle = /*((fabs(ball_angle)>=0.00001)?0.00001:*/env.home[r_number].ball.angle;
    if(fabs(ball_angle)<=0.00001)ball_angle=0.00001;
//    double c_ball_dis = sqrt((ball_dis*ball_dis)+(half_robot*half_robot) - (2*ball_dis*half_robot*cos(ball_angle*deg2rad)));
    int sign=1;
    if(ball_angle<0)sign=(-1);
//    double c_ball_angle = sign*(180- rad2deg*acos(((c_ball_dis*c_ball_dis)+(half_robot*half_robot)-(ball_dis*ball_dis))/(2*c_ball_dis*half_robot)));
    double c_ball_dis = ball_dis;
    double c_ball_angle = ball_angle;

    if(ball_angle==0.00001)c_ball_angle=ball_angle;

    double goal_dis = env.home[r_number].goal.distance;
    double goal_angle = env.home[r_number].goal.angle;
    double opgoal_dis = env.home[r_number].op_goal.distance;
    double opgoal_angle = env.home[r_number].op_goal.angle;

    printf("distance_br = %f\n", ball_dis);
    // here's the strange differnce
    double temp;
    temp = goal_dis;
    goal_dis = opgoal_dis;
    opgoal_dis = temp;
    temp = goal_angle;
    goal_angle = opgoal_angle;
    opgoal_angle = temp;

    double ball_x = -c_ball_dis*sin(c_ball_angle*deg2rad);
    double ball_y = c_ball_dis*cos(c_ball_angle*deg2rad);
    double goal_x = -goal_dis*sin(goal_angle*deg2rad);
    double goal_y = goal_dis*cos(goal_angle*deg2rad);

    double alpha = goal_angle - c_ball_angle;
    double beta = atan2( 0.73 , c_ball_dis) * rad2deg;

    Vector2d vectorbr(ball_x, ball_y);
    Vector2d vectordr(goal_x, goal_y);

    if(beta < alpha){
        alpha = beta;
    }else if(alpha < -beta){
        alpha = -beta;
    }

    double rotAngle = -alpha;

    Rotation2Dd rot( rotAngle * deg2rad);
    Vector2d vectornt = rot * vectorbr;

    env.home[r_number].v_x =vectornt(0);
    env.home[r_number].v_y =vectornt(1);
    env.home[r_number].v_yaw = goal_angle;
}

void FIRA_pathplan_class::strategy_Attack(int Robot_index){
    int r_number = Robot_index;
    double ball_dis = env.home[r_number].ball.distance;
    double ball_angle = /*((fabs(ball_angle)>=0.00001)?0.00001:*/env.home[r_number].ball.angle;
    if(fabs(ball_angle)<=0.00001)ball_angle=0.00001;
 //   double c_ball_dis = sqrt((ball_dis*ball_dis)+(half_robot*half_robot) - (2*ball_dis*half_robot*cos(ball_angle*deg2rad)));
    int sign=1;
    if(ball_angle<0)sign=(-1);
//    double c_ball_angle = sign*(180- rad2deg*acos(((c_ball_dis*c_ball_dis)+(half_robot*half_robot)-(ball_dis*ball_dis))/(2*c_ball_dis*half_robot)));
    double c_ball_dis = ball_dis;
    double c_ball_angle = ball_angle;

    if(ball_angle==0.00001)c_ball_angle=ball_angle;
    double goal_angle = env.home[r_number].goal.angle;
    double goal_dis = env.home[r_number].goal.distance;
    double alpha = goal_angle-c_ball_angle;
    ////========== normalization angle to -180~180 ==========
      if(alpha>180)
          alpha-=360;
      else if(alpha<-180)
          alpha+=360;
    ////========== normalization end ==========
    Vector2d vectornt(-1.42*sin(c_ball_angle*deg2rad),1.42*cos(c_ball_angle*deg2rad));
    double goal_max = env.home[r_number].goal_edge.max;
    double goal_min = env.home[r_number].goal_edge.min;
    double gla_angle = env.home[r_number].goal_large_area.angle;
    if(goal_angle>-90&&goal_angle<90){
        if(goal_min<0&&goal_max>=0){
            if(fabs(gla_angle)<=1){
                printf("i can shoot \n");
                shoot = SPlanning_Velocity[10];
            }
        }/*
        if((goal_edge1<0&&goal_edge2>=0)||(goal_edge1>=0&&goal_edge2<0)){
            printf("i can shoot \n");
            shoot = SPlanning_Velocity[10];
            env.home[r_number].v_x =vectornt(0)*1000;
            env.home[r_number].v_y =vectornt(1)*1000;
            env.home[r_number].v_yaw = goal_angle*2;
        }*/
        env.home[r_number].v_x =vectornt(0)*1000;
        env.home[r_number].v_y =vectornt(1)*1000;
        env.home[r_number].v_yaw = gla_angle*2;
//    }
//    if(fabs(goal_angle)<10){
//        shoot = SPlanning_Velocity[10];
//        env.home[r_number].v_x =vectornt(0)*1000;
//        env.home[r_number].v_y =vectornt(1)*1000;
//        env.home[r_number].v_yaw = goal_angle*2;
    }else{
        shoot = 0;
        env.home[r_number].v_x =vectornt(0)*1000;
        env.home[r_number].v_y =vectornt(1)*1000;
        env.home[r_number].v_yaw = gla_angle*2;
    }
}

void FIRA_pathplan_class::strategy_Shoot_Attack(int Robot_index){

    int r_number = Robot_index;
    double ball_dis = env.home[r_number].ball.distance;
    double ball_angle = /*((fabs(ball_angle)>=0.00001)?0.00001:*/env.home[r_number].ball.angle;
    if(fabs(ball_angle)<=0.00001)ball_angle=0.00001;
 //   double c_ball_dis = sqrt((ball_dis*ball_dis)+(half_robot*half_robot) - (2*ball_dis*half_robot*cos(ball_angle*deg2rad)));
    int sign=1;
    if(ball_angle<0)sign=(-1);
//    double c_ball_angle = sign*(180- rad2deg*acos(((c_ball_dis*c_ball_dis)+(half_robot*half_robot)-(ball_dis*ball_dis))/(2*c_ball_dis*half_robot)));
    double c_ball_dis = ball_dis;
    double c_ball_angle = ball_angle;

    if(ball_angle==0.00001)c_ball_angle=ball_angle;
    double goal_angle = env.home[r_number].goal.angle;
    double goal_dis = env.home[r_number].goal.distance;
    double alpha = goal_angle-c_ball_angle;

    double vectordr_x = -goal_dis * sin(goal_angle * deg2rad);
    double vectordr_y = goal_dis * cos(goal_angle * deg2rad);
    ////========== normalization angle to -180~180 ==========
      if(alpha>180)
          alpha-=360;
      else if(alpha<-180)
          alpha+=360;
    ////========== normalization end ==========
    Vector2d vectornt(-1.42*sin(c_ball_angle*deg2rad),1.42*cos(c_ball_angle*deg2rad));

    double goal_max = env.home[r_number].goal_edge.max;
    double goal_min = env.home[r_number].goal_edge.min;
    double gla_angle = env.home[r_number].goal_large_area.angle;
    printf("gla_angle=%f\n",gla_angle);
    if(goal_angle>-90&&goal_angle<90){
        if(goal_min<0&&goal_max>=0){
            if(fabs(gla_angle)<=1){
                printf("i can shoot \n");
                shoot = SPlanning_Velocity[10];
            }
        }
        env.home[r_number].v_x =0;
        env.home[r_number].v_y =0;
        env.home[r_number].v_yaw = gla_angle*2;
//    }
//    if(fabs(goal_angle)<10){
//        shoot = SPlanning_Velocity[10];
//        env.home[r_number].v_x =vectornt(0)*1000;
//        env.home[r_number].v_y =vectornt(1)*1000;
//        env.home[r_number].v_yaw = goal_angle*2;
    }else{
        shoot = 0;
        env.home[r_number].v_x =0;
        env.home[r_number].v_y =0;
        env.home[r_number].v_yaw = gla_angle*2;
    }
}
// useless play
void FIRA_pathplan_class::strategy_KO5_Attack(int Robot_index){

    //======parameter=======
    double spdParam = 2.5;  // bigger then the force of x direction will be bigger
    double EdgeVal = 6.5;
    double spd_y = 0.5;
    //======================

    int r_number = Robot_index;

    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;

    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;

    if(fabs(angle_br) <= 0.00001) angle_br=0.00001;
    double c_distance_br = sqrt((distance_br * distance_br)+(half_robot*half_robot)
                                - (2*distance_br * half_robot*cos(angle_br*deg2rad)));
    int sign = 1;
    if(angle_br < 0) sign=(-1);
    double c_angle_br = sign*(180- rad2deg*acos(((c_distance_br * c_distance_br)+(half_robot*half_robot)-(distance_br * distance_br))/(2*c_distance_br * half_robot)));
    if(angle_br==0.00001) c_angle_br = angle_br;

    double angle_bd = angle_dr - c_angle_br;
    double beta = atan2( 0.7 , c_distance_br) * rad2deg;

    double vectorbr_x = -c_distance_br * sin(c_angle_br * deg2rad);
    double vectorbr_y = c_distance_br * cos(c_angle_br * deg2rad);
    double vectordr_x = -distance_dr * sin(angle_dr * deg2rad);
    double vectordr_y = distance_dr * cos(angle_dr * deg2rad);
    double op_vectordr_x = -op_distance_dr * sin(op_angle_dr * deg2rad);
    double op_vectordr_y = op_distance_dr * cos(op_angle_dr * deg2rad);

    Vector2d vectorbr(vectorbr_x, vectorbr_y);
    Vector2d vectordr(vectordr_x, vectordr_y);

    double crossJudge = vectordr_x * op_vectordr_y - vectordr_y * op_vectordr_x;

    if(beta < angle_bd){
        angle_bd = beta;
    }else if(angle_bd < -beta){
        angle_bd = -beta;
    }

    double rotAngle = -angle_bd;

    Rotation2Dd rot( rotAngle * deg2rad);
    Vector2d vectornt = rot * vectorbr;

    //==============================KO5_Attack========================================

    double spdCtrl_x = spdParam / distance_dr;

    if(fabs(crossJudge) < EdgeVal){    // if not on the edge of the field
        if(crossJudge >= 0){     // on the right field
            env.home[r_number].v_x = -spdCtrl_x;
            env.home[r_number].v_y = -spd_y;
        }else if(crossJudge < 0){
            env.home[r_number].v_x = spdCtrl_x;
            env.home[r_number].v_y = -spd_y;
        }
    }else{
        env.home[r_number].v_x = 0;
        env.home[r_number].v_y = 0;
    }
    env.home[r_number].v_yaw = op_angle_dr;

//    printf("angle_opd2d = %f\t", angle_opd2d);
//    printf("crossJudge = %f\n", crossJudge);
}

void FIRA_pathplan_class::strategy_Zone_Attack(int Robot_index){
//--------------------parameter--------------------------------//
    double distance_backward = Zone_Attack[0];//0.6
    double distance_forward = Zone_Attack[1];//1.4
//-------------------------------------------------------------//
    int r_number = Robot_index;

    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;

    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;

    if(fabs(angle_br) <= 0.00001) angle_br=0.00001;
    double c_distance_br = sqrt((distance_br * distance_br)+(half_robot*half_robot)
                                - (2*distance_br * half_robot*cos(angle_br*deg2rad)));
    int sign = 1;
    if(angle_br < 0) sign=(-1);
    double c_angle_br = sign*(180- rad2deg*acos(((c_distance_br * c_distance_br)+(half_robot*half_robot)-(distance_br * distance_br))/(2*c_distance_br * half_robot)));
    if(angle_br==0.00001) c_angle_br = angle_br;

    double angle_bd = angle_dr - c_angle_br;
    double beta = atan2( 0.7 , c_distance_br) * rad2deg;

    double vectorbr_x = -c_distance_br * sin(c_angle_br * deg2rad);
    double vectorbr_y = c_distance_br * cos(c_angle_br * deg2rad);
    double vectordr_x = -distance_dr * sin(angle_dr * deg2rad);
    double vectordr_y = distance_dr * cos(angle_dr * deg2rad);
    Vector2d vectorbr(vectorbr_x, vectorbr_y);
      if(beta < angle_bd){
        angle_bd = beta;
    }else if(angle_bd < -beta){
        angle_bd = -beta;
    }
    double rotAngle = -angle_bd;

    Rotation2Dd rot( rotAngle * deg2rad);
    Vector2d vectornt = rot * vectorbr;

    if(distance_dr < distance_backward){
        env.home[r_number].v_x = 0;
        env.home[r_number].v_y = -1;
         env.home[r_number].v_yaw = 0;
    }else if(distance_dr >= distance_forward){
        env.home[r_number].v_x = vectordr_x;
        env.home[r_number].v_y = vectordr_y;
        env.home[r_number].v_yaw = angle_dr * 2/3/*5*/;
    }
    shoot = SPlanning_Velocity[10];
}


void FIRA_pathplan_class::strategy_typeS_Attack(int Robot_index){

    //================parameter========================
    double cycle = TypeS_Attack[0];//1.5
    double speedMin = TypeS_Attack[1];//0.2
    //=================================================
    int r_number = Robot_index; 	// assign which robot you are going to use

    double distance_br = env.home[r_number].ball.distance;	// distance between the ball and the robot

    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;

    if(fabs(angle_br) <= 0.00001) angle_br=0.00001;
    double c_distance_br = sqrt((distance_br * distance_br)+(half_robot*half_robot)
                                - (2*distance_br * half_robot*cos(angle_br*deg2rad)));	// distance between robot's head and the ball

    int sign = 1;
    if(angle_br < 0) sign=(-1);
    double c_angle_br = sign*(180- rad2deg*acos(((c_distance_br * c_distance_br)+(half_robot*half_robot)-(distance_br * distance_br))/(2*c_distance_br * half_robot)));
    if(angle_br==0.00001) c_angle_br = angle_br;

    double angle_bd = angle_dr - c_angle_br;
    double beta = atan2( 0.7 , c_distance_br) * rad2deg;    // a self define angle which improves the path to pursue the ball

    double vectorbr_x = -c_distance_br * sin(c_angle_br * deg2rad);
    double vectorbr_y = c_distance_br * cos(c_angle_br * deg2rad);

    Vector2d vectorbr(vectorbr_x, vectorbr_y);

    if(beta < angle_bd){    // compare which angle we are going to use
        angle_bd = beta;
    }else if(angle_bd < -beta){
        angle_bd = -beta;
    }

    double rotAngle = -angle_bd;

    Rotation2Dd rot( rotAngle * deg2rad);
    Vector2d vectornt = rot * vectorbr;

    /*============================TYPE S attack=============================== */

    static int iCondition = 0;  // condition when the ball is on the right's field and the left's
    static bool loopEnd = true;
    double op_angle_dr = env.home[r_number].op_goal.angle;

    //---------------------------Time---------------------------//
    static ros::Time begin = ros::Time::now();
    ros::Time current = ros::Time::now();
    double begin_time = (double)(begin.sec+(double)begin.nsec/1000000000);
    double current_time =(double)(current.sec+(double)current.nsec/1000000000);
    double exec_time = current_time - begin_time;
    //-----------------------------------------------------------
    double multiple = 180 / cycle;

    if(loopEnd == true){
        // condition when the robot is holding the ball and facing the door
        loopEnd = false;
        if(op_angle_dr >= 0){
            iCondition = 1;
        }else if(op_angle_dr < 0){
            iCondition = 2;
        }
    }

    if(iCondition == 1){   // on the right field
        if(exec_time > cycle * 2){  // reset the counter and the condition
            begin.sec = current.sec;
            begin.nsec = current.nsec;
//            vectornt = rot * vectorbr;
        }else if(exec_time > cycle){
            vectornt(0) = (-sin(exec_time * multiple * deg2rad));
        }else{  // go left first
            vectornt(0) = (-sin(exec_time * multiple * deg2rad));
        }
        if(vectornt(0) < speedMin && vectornt(0) >= 0)   // speed adjustment
            vectornt(0) = speedMin;
        else if(vectornt(0) > -speedMin && vectornt(0) < 0)
            vectornt(0) = -speedMin;
    }else if(iCondition == 2){ // on the left field
        if(exec_time > cycle * 2){
            begin.sec = current.sec;
            begin.nsec = current.nsec;
//            vectornt = rot * vectorbr;
        }else if(exec_time > cycle){
            vectornt(0) = sin(exec_time * multiple * deg2rad);
        }else{  // go right first
            vectornt(0) = sin(exec_time * multiple * deg2rad);
        }
        if(vectornt(0) < speedMin && vectornt(0) >= 0)
            vectornt(0) = speedMin;
        else if(vectornt(0) > -speedMin && vectornt(0) < 0)
            vectornt(0) = -speedMin;
    }
//    printf("exec_time = %lf\tvectornt = %lf\n", exec_time, vectornt(0));
//    printf("op_angle_dr=%lf\n",op_angle_dr);
    if(fabs(angle_dr) < 7){
        shoot = SPlanning_Velocity[10];
    }else{
        shoot = 0;
    }
    env.home[r_number].v_x = vectornt(0);
    env.home[r_number].v_y = vectornt(1)*2;
    env.home[r_number].v_yaw = angle_dr;
}

void FIRA_pathplan_class::strategy_typeU_Attack(int Robot_index){
//--------------------parameter--------------------------------//
    double cycle = TypeU_Attack[0];//0.8
    double circle_y_speed = TypeU_Attack[1];//1.0
    int circle_size_right = TypeU_Attack[2];//-90
    int circle_size_left = TypeU_Attack[3];//90
    double distance_br_min =TypeU_Attack[4];//0.25
    double distance_dr_min =TypeU_Attack[5];//0.3
    int angle_br_min =TypeU_Attack[6];//20
    int angle_dr_min =TypeU_Attack[7];//30
//----------------------------------------------------------------//
    int r_number = Robot_index;

    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double distance_bd = distance_dr - distance_br;

    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;

    if(fabs(angle_br) <= 0.00001) angle_br=0.00001;
    double c_distance_br = sqrt((distance_br * distance_br)+(half_robot*half_robot) - (2*distance_br * half_robot*cos(angle_br*deg2rad)));
    int sign=1;
    if(angle_br < 0) sign=(-1);
    double c_angle_br = sign*(180- rad2deg*acos(((c_distance_br * c_distance_br)+(half_robot*half_robot)-(distance_br * distance_br))/(2*c_distance_br * half_robot)));
    if(angle_br==0.00001) c_angle_br = angle_br;

    double angle_bd = angle_dr - c_angle_br;
    double beta = atan2( 0.7 , c_distance_br) * rad2deg;

    double vectorbr_x = -c_distance_br * sin(c_angle_br * deg2rad);
    double vectorbr_y = c_distance_br * cos(c_angle_br * deg2rad);
    double vectordr_x = -distance_dr * sin(angle_dr * deg2rad);
    double vectordr_y = distance_dr * cos(angle_dr * deg2rad);

    Vector2d vectorbr(vectorbr_x, vectorbr_y);
    Vector2d vectordr(vectordr_x, vectordr_y);

    if(beta < angle_bd){
        angle_bd = beta;
    }else if(angle_bd < -beta){//rush door
        angle_bd = -beta;
    }
    double rotAngle = -angle_bd;

    Rotation2Dd rot( rotAngle * deg2rad);
    Vector2d vectornt = rot * vectorbr;

    double op_angle_dr = env.home[r_number].op_goal.angle;

//---------------------------------Time------------------------------------------//
    static ros::Time begin = ros::Time::now();
    ros::Time Current = ros::Time::now();
    double begin_time = (double)(begin.sec+(double)begin.nsec/1000000000);
    double Current_time =(double)(Current.sec+(double)Current.nsec/1000000000);
//-------------------------------------------------------------------------------//
    static int iCondition=0;
    static bool bool1 = false;
   if((Current_time - begin_time) > cycle){
       bool1 = true;
   }
   if(bool1 == true){
     if(iCondition == 0){
         env.home[r_number].v_x = vectorbr_x;
         env.home[r_number].v_y = vectorbr_y;
         env.home[r_number].v_yaw = angle_br;
         if(fabs(c_angle_br) <= angle_br_min && distance_br <= distance_br_min){
            if(angle_dr < 0){
                if((angle_dr-op_angle_dr)<-180){
                    iCondition =2;
                }else{
                    iCondition =1;
                }
            }else{
                if((angle_dr-op_angle_dr)>-180 && (angle_dr-op_angle_dr) < 180){
                    iCondition =2;
                }else{
                    iCondition =1;
                }
            }
         }
      }
   }

    if(iCondition == 1){
        if(distance_br < distance_br_min){
            env.home[r_number].v_y = circle_y_speed;             //change cycle big or small
            env.home[r_number].v_yaw = circle_size_right;        //change cycle big or small
            if(fabs(angle_dr) <= angle_dr_min && c_distance_br <= distance_br_min){ //chase goal
                    env.home[r_number].v_x = vectordr_x;
                    env.home[r_number].v_y = vectordr_y;
                    env.home[r_number].v_yaw = angle_dr ;
            }
        }else{
            iCondition = 0;
        }
    }else if(iCondition == 2){
        if(distance_br < distance_br_min){
            env.home[r_number].v_y = circle_y_speed;             //change cycle big or small
            env.home[r_number].v_yaw = circle_size_left;         //change cycle big or small
            if(fabs(angle_dr) <=angle_dr_min && c_distance_br <= distance_br_min){//chase goal
                    env.home[r_number].v_x = vectordr_x;
                    env.home[r_number].v_y = vectordr_y;
                    env.home[r_number].v_yaw = angle_dr ;

            }
        }else{
            iCondition = 0;
        }
    }
    shoot = 0;
}
void FIRA_pathplan_class::strategy_Dorsad_Attack(int Robot_index){
    int r_number = Robot_index;

    //-----------parameter-----------
    double deceleration = Dorsad_Attack[0];//8    // level of deceleration when triggering the dorsad strategy
    double angle2opr1 = Dorsad_Attack[1];//90     // angle to turn away from enemy
    double angle2opr2 = Dorsad_Attack[2];//180
    double distance2turn1 = Dorsad_Attack[3];//2   // condition to turn away from enemy (by distance to enemy's robot)
    double distance2turn2 = Dorsad_Attack[4];//1
    double distance2rush = Dorsad_Attack[5];//2   // condition to ignore the dorsad strategy (by distance to our goal)
    //-------------------------------

    //------in simulator-------------
//    Vector3D Robot = env.home[r_number].pos;
//    Vector3D Enemy = env.home[2].pos;
//    double vectorOpr_x = Enemy.x - Robot.x;
//    double vectorOpr_y = Enemy.y - Robot.y;
//    double phi_Opr = atan2(vectorOpr_y, vectorOpr_x) * rad2deg;

//    double op_distance_rr = hypot(vectorOpr_x,vectorOpr_y);    // distance between robot and enemy's robot
//    double op_angle_rr = phi_Opr - env.home[r_number].rotation;     // angle between robot to enemy's robot and robot head's direction

//    // normalize
//    if(op_angle_rr > 180)
//    {
//        op_angle_rr = op_angle_rr - 360;
//    }else if(op_angle_rr < -180){
//        op_angle_rr = op_angle_rr + 360;
//    }
    //---------on robot---------
//    double op_distance_rr = double(env.mindis[0]) / 100;
//    int op_angle_rr = env.blackangle[0];
    double op_distance_rr = env.Support_Obstacle_distance;
    int op_angle_rr = env.Support_Obstacle_angle;
    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;

    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;

    if(fabs(angle_br) <= 0.00001) angle_br=0.00001;
    double c_distance_br = sqrt((distance_br * distance_br)+(half_robot*half_robot)
                                - (2*distance_br * half_robot*cos(angle_br*deg2rad)));
    int sign = 1;
    if(angle_br < 0) sign=(-1);
    double c_angle_br = sign*(180- rad2deg*acos(((c_distance_br * c_distance_br)+(half_robot*half_robot)-(distance_br * distance_br))/(2*c_distance_br * half_robot)));
    if(angle_br==0.00001) c_angle_br = angle_br;

    double vectordr_x = -distance_dr * sin(angle_dr * deg2rad);
    double vectordr_y = distance_dr * cos(angle_dr * deg2rad);


    //-----------Dorsad Attack------------------
    double angle_vertical;
    double angle_dorsad;
    double opr_angle_dr = op_angle_rr - angle_dr;
    double angle_Speed;

    // judge which way to turn depend on enemy's direction
    if(op_angle_rr - angle_dr > 0){
        angle_vertical = op_angle_rr - angle2opr1;
        angle_dorsad = op_angle_rr - angle2opr2;
    }else{
        angle_vertical = op_angle_rr + angle2opr1;
        angle_dorsad = op_angle_rr + angle2opr2;
    }
    // normalize
    if(angle_vertical > 180){
        angle_vertical -= 360;
    }else if(angle_vertical < -180){
        angle_vertical += 360;
    }
    if(angle_dorsad > 180){
        angle_dorsad -= 360;
    }else if(angle_dorsad < -180){
        angle_dorsad += 360;
    }
    if(opr_angle_dr > 180){
        opr_angle_dr -= 360;
    }else if(opr_angle_dr < -180){
        opr_angle_dr += 360;
    }

    if(distance_dr < distance2rush){
//        printf("RUSH!!!!!\n");
       if(op_angle_rr > 0){
           if(angle_dr > 0){    // avoid facing the enemy
               angle_Speed = -angle_dr;  //rush to the door
           }else{
               angle_Speed = angle_dr;
           }
       }else{
           if(angle_dr > 0){
               angle_Speed = angle_dr;  //rush to the door
           }else{
               angle_Speed = -angle_dr;
           }
       }
       env.home[r_number].v_x = vectordr_x;
       env.home[r_number].v_y = vectordr_y;
       std::cout << "flag1\t" << vectordr_x << std::endl;
    }else if(op_distance_rr < distance2turn2){
       angle_Speed = angle_dorsad;// * 2/3;
       env.home[r_number].v_x = vectordr_x * (op_distance_rr / deceleration);
       env.home[r_number].v_y = vectordr_y * (op_distance_rr / deceleration);
       std::cout << "flag2\t" << op_distance_rr << std::endl;
    }else if(op_distance_rr < distance2turn1){
       angle_Speed = angle_vertical;/* * 2/3*/
       env.home[r_number].v_x = vectordr_x * (op_distance_rr / deceleration);
       env.home[r_number].v_y = vectordr_y * (op_distance_rr / deceleration);
       std::cout << "flag3\t" << op_distance_rr << std::endl;
    }else{
       angle_Speed = angle_dr; //* 2/3;
       env.home[r_number].v_x = vectordr_x;
       env.home[r_number].v_y = vectordr_y;
       std::cout << "flag4\t" << vectordr_x << std::endl;
    }
    env.home[r_number].v_yaw = angle_Speed;

    double goal_max = env.home[r_number].goal_edge.max;
    double goal_min = env.home[r_number].goal_edge.min;
    double gla_angle = env.home[r_number].goal_large_area.angle;
    if(angle_dr>-90&&angle_dr<90){
        if(goal_min<0&&goal_max>=0){
            if(fabs(gla_angle)<=1){
                printf("i can shoot \n");
                shoot = SPlanning_Velocity[10];
            }
        }
//    }
//    if(fabs(goal_angle)<10){
//        shoot = SPlanning_Velocity[10];
//        env.home[r_number].v_x =vectornt(0)*1000;
//        env.home[r_number].v_y =vectornt(1)*1000;
//        env.home[r_number].v_yaw = goal_angle*2;
    }else{
        shoot = 0;
    }
}

void FIRA_pathplan_class::strategy_SideSpeedUp(int Robot_index){
    //-----------parameter--------------------------------------
    double beta_param = Side_Speed_Up[0]; //0.73
    double const const_angle_mult = Side_Speed_Up[1];//1.2
    double const const_dis = Side_Speed_Up[2];//0.7
    double const const_mult = Side_Speed_Up[3];//5.0
    double const const_only = Side_Speed_Up[4];//2.2*0.8*10=17.6
    //-----------------------------------------------------------
    int r_number = Robot_index;
    double ball_dis = env.home[r_number].ball.distance;
    double ball_angle = /*((fabs(ball_angle)>=0.00001)?0.00001:*/env.home[r_number].ball.angle;
    if(fabs(ball_angle)<=0.00001)ball_angle=0.00001;
    double c_ball_dis = sqrt((ball_dis*ball_dis)+(half_robot*half_robot) - (2*ball_dis*half_robot*cos(ball_angle*deg2rad)));
    int sign=1;
    if(ball_angle<0)sign=(-1);
    double c_ball_angle = sign*(180- rad2deg*acos(((c_ball_dis*c_ball_dis)+(half_robot*half_robot)-(ball_dis*ball_dis))/(2*c_ball_dis*half_robot)));
    if(ball_angle==0.00001)c_ball_angle=ball_angle;

    double goal_dis = env.home[r_number].goal.distance;
    double goal_angle = env.home[r_number].goal.angle;
    double ball_x = -c_ball_dis*sin(c_ball_angle*deg2rad);
    double ball_y = c_ball_dis*cos(c_ball_angle*deg2rad);
    double alpha = goal_angle - c_ball_angle;
    double beta = atan2( beta_param , c_ball_dis) * rad2deg;

    Vector2d vectorbr(ball_x, ball_y);
        if(beta < alpha){
            alpha = beta;
        }else if(alpha < -beta){
            alpha = -beta;
        }

        double rotAngle = -(alpha*const_angle_mult);
        Rotation2Dd rot( rotAngle * deg2rad);
        Vector2d vectornt = rot * vectorbr;
         double const_dis_x,const_dis_y;
         ////======================================================
         double const_vector_x,const_vector_y;
         double vectornt_angle=atan2(vectornt(1),vectornt(0));
             const_vector_x=const_only*vectornt(0)/hypot(vectornt(1),vectornt(0));
             const_vector_y=const_only*vectornt(1)/hypot(vectornt(1),vectornt(0));
        env.home[r_number].v_x =const_vector_x;
        env.home[r_number].v_y =const_vector_y;
        env.home[r_number].v_yaw =goal_angle*2;
        shoot = 0;
}

void FIRA_pathplan_class::strategy_Support(int Robot_index){
//==========one position===========
//    int r_number = Robot_index;
//    int r_number1;
//    if(r_number == 1) r_number1 = 2;
//    else r_number1 = 1;

//    Vector3D robot = env.home[r_number].pos;
//    Vector3D robot1 = env.home[r_number1].pos;

//    double target_x;
//    double target_y;
//    if(robot1.x >0){
//        target_x = -2;
//        target_y = 1;
//    }else{
//        target_x = 2;
//        target_y = 1;
//    }
//    double v_x = target_x - robot.x;
//    double v_y = target_y - robot.y;

//    env.home[Robot_index].v_x = v_x;
//    env.home[Robot_index].v_y = v_y;

//==========
    double angle,v_x,v_y;
    int goal_color;
    int r_number = Robot_index;
    Vector3D ball = env.currentBall.pos;
    Vector3D robot = env.home[r_number].pos;
    Vector3D goal ;
    Vector3D target;

    if(mTeam == Team_Blue){
        goal = env.yellow.pos;
        goal_color = Goal_Yellow;
        target.x = -1.5;
        if(robot.y>0){
            target.y = 1;
        }else{
            target.y = -1;
        }
    }else if (mTeam == Team_Yellow){
        goal = env.blue.pos;
        goal_color = Goal_Blue;
        target.x = 1.5;
        if(robot.y>0){
            target.y = -0.5;
        }else{
            target.y = -0.5;
        }
    }
    double Robot_head_x = robot.x + half_robot*cos(env.home[r_number].rotation*deg2rad);
    double Robot_head_y = robot.y + half_robot*sin(env.home[r_number].rotation*deg2rad);

    double vectorbr_x = ball.x - Robot_head_x;
    double vectorbr_y = ball.y - Robot_head_y;
    double vectordr_x = goal.x - Robot_head_x;
    double vectordr_y = goal.y - Robot_head_y;
    double vectortr_x = target.x - Robot_head_x;
    double vectortr_y = target.y - Robot_head_y;
    double vectorbd_x = ball.x - goal.x;
    double vectorbd_y = ball.y - goal.y;

    double dis_tr = hypot(vectortr_x,vectortr_y);
    double dis_br = hypot(vectorbr_x,vectorbr_y);
    double distance_dr = hypot(vectordr_x,vectordr_y);
    double distance_bd = hypot(vectorbd_x,vectorbd_y);
//    double distance_a  = hypot(distance_br,1);

 //    printf("dis_br = %lf\tdistance_dr = %lf\t",dis_br, distance_dr);
    double phi = atan2(vectordr_y , vectordr_x) * rad2deg;
//    double alpha = acos((vectorbr_x*vectordr_x+vectorbr_y*vectordr_y)/(dis_br*distance_dr))* rad2deg;
    double beta = atan2( 0.73 , dis_tr) * rad2deg;

    Vector2d vectorbr(ball.x-Robot_head_x, ball.y-Robot_head_y );
    Vector2d vectortr(target.x - Robot_head_x, target.y - Robot_head_y);
 //   if(vectorbr(0)==0)vectorbr(0)=very_small;
 //   if(vectorbr(1)==0)vectorbr(1)=very_small;
 //   Vector2d vectordr(blueGoal.x-robot.x, blueGoal.y-robot.y);

//    if(beta < alpha){
//        alpha = beta;
//    }else if(alpha < -beta){
//        alpha = -beta;
//    }
//    double rotAngle = alpha;
    double rotAngle = beta;
    if(goal_color == Goal_Yellow){
        if(vectortr_y < 0)rotAngle = -beta;
    }
    if (goal_color == Goal_Blue){
        if(vectortr_y > 0)rotAngle = -beta;
    }

    Rotation2Dd rot( rotAngle * deg2rad);
    Vector2d vectornt = rot * vectortr;


 ///-------------------------------------------------
        angle = phi - env.home[r_number].rotation;
        if(angle>180)
        {
            angle = angle - 360;
        }else if(angle < -180){
            angle = angle + 360;
        }

        env.home[r_number].v_x = vectornt(0);
        env.home[r_number].v_y = vectornt(1);
        env.home[r_number].v_yaw = angle*5/*5*/;
        shoot = 0;

}

void FIRA_pathplan_class::strategy_Halt(int Robot_index){
    env.home[Robot_index].v_x = 0;
    env.home[Robot_index].v_y = 0;
    env.home[Robot_index].v_yaw = 0;
    shoot = 0;
    Begin_time = ros::Time::now().toSec();
    Current_time = ros::Time::now().toSec();
    loopEnd = true;
}

void FIRA_pathplan_class::strategy_PenaltyKick(int Robot_index){

////Original Start
//    Vector3D robot = env.home[Robot_index].pos;
//    Vector3D ball = env.currentBall.pos;
//    Vector3D goal = env.yellow.pos;

//    double Robot_head_x = robot.x + half_robot*cos(env.home[Robot_index].rotation*deg2rad);
//    double Robot_head_y = robot.y + half_robot*sin(env.home[Robot_index].rotation*deg2rad);

//    double vectorbr_x = ball.x - Robot_head_x;
//    double vectorbr_y = ball.y - Robot_head_y;

//    double vectordr_x = goal.x - Robot_head_x;
//    double vectordr_y = goal.y - Robot_head_y;

//    double dis_br = hypot(vectorbr_x,vectorbr_y);
//    double dis_dr = hypot(vectordr_x,vectordr_y);

//    double v_x = vectordr_x*6;
//    double v_y = vectordr_y*6;

//    double phi = atan2(vectordr_y , vectordr_x) * rad2deg;
//    double angle = phi - env.home[Robot_index].rotation;
//    if(angle>180)
//    {
//        angle = angle - 360;
//    }else if(angle < -180){
//        angle = angle + 360;
//    }

//    if((fabs(robot.y) <= 1.5 && dis_br<=0.5)){
//        env.home[1].v_x=v_x;
//        env.home[1].v_y=v_y;
//        env.home[1].v_yaw=angle;
//    }else{
//        strategy_Support(Robot_index);
//    }
////Original end

////goal angle start (not stable)
//    double goal_angle = env.home[Robot_index].goal.angle;
//    static double first_goal_angle = goal_angle;
//    double degree = Penalty_Kick[0];
//    static double des_angle = first_goal_angle + degree;
//    static double last_degree = degree;
//    static int shoot_count = 0;
//    if(last_degree != degree){
//        first_goal_angle = goal_angle;
//        des_angle = first_goal_angle + degree;
//        if(des_angle > 180){
//            des_angle = des_angle - 360;
//        }
//        if(des_angle < -180){
//            des_angle = des_angle + 360;
//        }
//        last_degree = degree;
//        shoot_count = 1;
//    }
//    double yaw_speed=goal_angle-des_angle;

//    if(yaw_speed> 180){
//        yaw_speed = yaw_speed - 360;
//    }
//    if(yaw_speed < -180){
//        yaw_speed = yaw_speed + 360;
//    }
//    env.home[Robot_index].v_yaw = (yaw_speed)*2;
//    printf("goal_angle=%f\n",goal_angle);
//    printf("des_angle=%f\n",des_angle);
//    printf("goal_angle-des_angle=%f\n",goal_angle-des_angle);
//    printf("last_degree=%f\n",last_degree);
//    printf("degree=%f\n",degree);
//    if(fabs(des_angle-goal_angle)<=7){
//        if(shoot_count == 1){
//            shoot = 50;
//            shoot_count = 0;
//        }
//    }else{
//        shoot = 0;
//    }

////goal angle end
    double goal_max = env.home[r_number].goal_edge.max;
    double goal_min = env.home[r_number].goal_edge.min;
    double gla_angle = env.home[r_number].goal_large_area.angle;
    static double first_right_goal_angle = right_goal_angle;
    double degree = Penalty_Kick[0];
    static double des_angle = first_right_goal_angle + degree;
    static double last_degree = degree;
    static int shoot_count = 1;
    double yaw_speed=144;
    int action = Penalty_Kick[1];
    switch(action){
        case 0:
            if(last_degree != degree){
                first_right_goal_angle = right_goal_angle;
                des_angle = first_right_goal_angle + degree;
                if(des_angle > 180){
                    des_angle = des_angle - 360;
                }
                if(des_angle < -180){
                    des_angle = des_angle + 360;
                }
                last_degree = degree;
                shoot_count = 1;
            }
            yaw_speed=right_goal_angle-des_angle;

            if(yaw_speed > 180){
                yaw_speed = yaw_speed - 360;
            }
            if(yaw_speed < -180){
                yaw_speed = yaw_speed + 360;
            }
            env.home[Robot_index].v_yaw = yaw_speed*2;
            printf("action=%d\n",action);
            printf("right_goal_angle=%f\n",right_goal_angle);
            printf("left_goal_angle=%f\n",left_goal_angle);
            printf("des_angle=%f\n",des_angle);
            printf("right_goal_angle-des_angle=%f\n",right_goal_angle-des_angle);
            printf("last_degree=%f\n",last_degree);
            printf("degree=%f\n",degree);
            if(fabs(des_angle-right_goal_angle)<=7){
                if(shoot_count == 1){
                    shoot = SPlanning_Velocity[10];
                    shoot_count = 0;
                }
            }else{
                shoot = 0;
            }
            break;
        case 1:
            des_angle = left_goal_angle - 20;
            if(des_angle < 15){
                env.home[Robot_index].v_yaw = (yaw_speed)/4;
            }else{
                env.home[Robot_index].v_yaw = yaw_speed;
            }
            printf("action=%d\n",action);
            printf("right_goal_angle=%f\n",right_goal_angle);
            printf("left_goal_angle=%f\n",left_goal_angle);
            printf("des_angle=%f\n",des_angle);
            printf("rbdistance=%f\n",env.home[Robot_index].ball.distance);
            printf("rbangle=%f\n",env.home[Robot_index].ball.angle);
            if(des_angle <= 0){
                env.home[Robot_index].v_yaw = 0;
                if(env.home[Robot_index].ball.distance < 0.35 /*&& fabs(env.home[Robot_index].ball.angle)<5*/ ){
                    shoot = SPlanning_Velocity[10];
                }
            }else{
                shoot = 0;
            }
            break;
        case 2:
            des_angle = right_goal_angle + 20;
            if(des_angle > -15){
                env.home[Robot_index].v_yaw = (-yaw_speed)/4;
            }else{
                env.home[Robot_index].v_yaw = -yaw_speed;
            }
            printf("action=%d\n",action);
            printf("right_goal_angle=%f\n",right_goal_angle);
            printf("left_goal_angle=%f\n",left_goal_angle);
            printf("des_angle=%f\n",des_angle);
            printf("rbdistance=%f\n",env.home[Robot_index].ball.distance);
            printf("rbangle=%f\n",env.home[Robot_index].ball.angle);
            if(des_angle >= 0){
                env.home[Robot_index].v_yaw = 0;
                if(env.home[Robot_index].ball.distance < 0.35 /*&& fabs(env.home[Robot_index].ball.angle)<5*/ ){
                    shoot = SPlanning_Velocity[10];
                }
            }else{
                shoot = 0;
            }
            break;
    }
}

void FIRA_pathplan_class::strategy_ThrowIn(int Robot_index){
//    int r_number = Robot_index;
//    int r_number1;
//    if(r_number == 1)r_number1 = 2;
//    else r_number = 1;
//    Vector3D ball = env.currentBall.pos;
//    Vector3D robot = env.home[r_number].pos;
//    Vector3D robot1 = env.home[r_number1].pos;


//    double Robot_head_x = robot.x + half_robot*cos(env.home[r_number].rotation*deg2rad);
//    double Robot_head_y = robot.y + half_robot*sin(env.home[r_number].rotation*deg2rad);
//    double Robot1_head_x = robot1.x + half_robot*cos(env.home[r_number1].rotation*deg2rad);
//    double Robot1_head_y = robot1.y + half_robot*sin(env.home[r_number1].rotation*deg2rad);


//    double vector1br_x = ball.x - Robot1_head_x;
//    double vector1br_y = ball.y - Robot1_head_y;

//    double vectorbr_x = ball.x - Robot_head_x;
//    double vectorbr_y = ball.y - Robot_head_y;


//    double vectorrr_x = Robot1_head_x - Robot_head_x;
//    double vectorrr_y = Robot1_head_y - Robot_head_y;

//    double dis_br = hypot(vectorbr_x,vectorbr_y);
//    double dis_rr = hypot(vectorrr_x,vectorrr_y);

//    double v_x = vectorrr_x *4;
//    double v_y = vectorrr_y *4;

//    double phi = atan2(vectorrr_y , vectorrr_x) * rad2deg;
//    double angle = phi - env.home[r_number].rotation;

//    if(angle>180)
//    {
//        angle = angle - 360;
//    }else if(angle < -180){
//        angle = angle + 360;
//    }
////    if((fabs(robot.y) >= 2 && dis_br<=1)){
////        env.home[1].v_x=v_x;
////        env.home[1].v_y=v_y;
////        env.home[1].v_yaw=angle;
////    }else{
////        strategy_Support(Robot_index);
////    }
    shoot = 0;
//    printf("shoot = 30\n");
}

void FIRA_pathplan_class::strategy_CornerKick(int Robot_index){
    //---------------parameter---------------------------
    double forwardV = Corner_Kick[0];//1.2
    double step1Ang = Corner_Kick[1];//60
    double step2Ang = Corner_Kick[2];//90
    double criticalAng = Corner_Kick[3];//50
    //--------------------------------------------------

    int r_number = Robot_index;   // assign which robot you are going to use

    double distance_br = env.home[r_number].ball.distance;  // distance between the ball and the robot

    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;

    if(fabs(angle_br) <= 0.00001) angle_br=0.00001;
    double c_distance_br = sqrt((distance_br * distance_br)+(half_robot*half_robot)
                                - (2*distance_br * half_robot*cos(angle_br*deg2rad)));  // distance between robot's head and the ball

    int sign=1;
    if(angle_br < 0) sign=(-1);
    double c_angle_br = sign*(180- rad2deg*acos(((c_distance_br * c_distance_br)+(half_robot*half_robot)-(distance_br * distance_br))/(2*c_distance_br * half_robot)));
    if(angle_br==0.00001) c_angle_br = angle_br;

    double angle_bd = angle_dr - c_angle_br;
    double beta = atan2( 0.7 , c_distance_br) * rad2deg;

    double vectorbr_x = -c_distance_br * sin(c_angle_br * deg2rad);
    double vectorbr_y = c_distance_br * cos(c_angle_br * deg2rad);

    Vector2d vectorbr(vectorbr_x, vectorbr_y);

    if(beta < angle_bd){
        angle_bd = beta;
    }else if(angle_bd < -beta){
        angle_bd = -beta;
    }

    double rotAngle = -angle_bd;

    Rotation2Dd rot( rotAngle * deg2rad);
    Vector2d vectornt = rot * vectorbr;

    /*------------------------Corner Kick---------------------------*/
    double op_angle_dr = env.home[r_number].op_goal.angle;
    static int iCondition = 0;  // condition when the ball is on the right's field or the left's

    /* use y (forward) velocity and the robot's target angle to make the robot go in a curve */

    if(op_angle_dr > 0 && loopEnd == true){   // robot kicks on the right field
        loopEnd = false;
        iCondition = 1;
    }else if(op_angle_dr < 0 && loopEnd == true){ // robot kicks on the left field
        loopEnd = false;
        iCondition = 2;
    }

    if(iCondition == 1){
        if(fabs(op_angle_dr) > criticalAng){   // when to enlarge angle in order to face the attacking door
            env.home[r_number].v_yaw = step2Ang;
        }else{
            env.home[r_number].v_yaw = step1Ang;
        }
        env.home[r_number].v_y = forwardV;
    }else if(iCondition == 2){
        if(fabs(op_angle_dr) > criticalAng){
            env.home[r_number].v_yaw = -step2Ang;
        }else{
            env.home[r_number].v_yaw = -step1Ang;
        }
        env.home[r_number].v_y = forwardV;
    }
    shoot = 0;
}

void FIRA_pathplan_class::strategy_AvoidBarrier(int Robot_index){
    shoot = 0;
}
void FIRA_pathplan_class::strategy_Escape_Attack(int r_number){
    double goal_angle = env.home[r_number].goal.angle;
    double goal_dis = env.home[r_number].goal.distance;
    std::vector<double>angle_avg;

    int effective_dis=125;
    double f_total[3]={0,0,0};

    double total_f_apf_goal=0;
    double total_f_apf=0;
    //double total_f_image=0;

    double f_g[3]={0,0,0};//x y z
    double f_image_r[3]={0,0,0};//x y z
    double e_image_r[3]={0,0,0};
    double f_image_total[2]={0,0};
    double apf_f[3]={0,0,0};//x y z
    double f_apf_total[2]={0,0};

    double f_r_angle=0;
    double f_g_angle=0;
    double Velocity_angle=0;

    int number_obstacle=0;

    float k_r[2]={1,1};
    float k_g[2]={1,1};
    float k_image[2]={0,0};
    float k_image_r[2]={0,0};

    //printf("===========f_g_cal=======\t%d\n\n",env.global_angle_end.size());
    ////////////////// f_g to unit vector//////////
    f_g[0]=goal_dis*-sin(goal_angle*deg2rad);
    f_g[1]=goal_dis*cos(goal_angle*deg2rad);
    //printf("goal_dis:%f\tgoal_angle:%f\n",goal_dis,goal_angle);

    if(goal_dis!=0){
        f_g[0]=f_g[0]/fabs(goal_dis);
        f_g[1]=f_g[1]/fabs(goal_dis);
    }

    //printf("f_g_x:%f\tf_g_y:%f\n\n",f_g[0],f_g[1]);

    //printf("==========apf_cal&&image_f============\n");

    angle_avg.clear();
    std::vector<double>().swap(angle_avg);

    //printf("%d\t%d\t%d\n",env.global_angle_end[0],env.global_angle_start[0],env.global_apf_dis[0]);
    if(env.global_angle_end[0]==1&&env.global_angle_start[0]==1&&env.global_apf_dis[0]==1){
        number_obstacle=0;
        for(int i=1;i<=env.global_angle_end.size()-1;i++){number_obstacle++;}//cal obstacle_number
        //printf("number_obstacle:%d\n",number_obstacle);

        for(int i=1;i<=env.global_angle_end.size()-1;i++){
            angle_avg.push_back((env.global_angle_end[i]+env.global_angle_start[i])/2);

            if(angle_avg[i-1]<=180){
                angle_avg[i-1]=angle_avg[i-1]+180;
            }else{
                angle_avg[i-1]=angle_avg[i-1]-180;
            }
            //printf("angle_start:%d\tangle_end:%d\tangle_avg:%f\n",env.global_angle_start[i],env.global_angle_end[i],angle_avg[i-1]);
            apf_f[0]=-sin(angle_avg[i-1]*deg2rad);//x
            apf_f[1]=cos(angle_avg[i-1]*deg2rad);//y
            f_apf_total[0]=f_apf_total[0]+(effective_dis-env.global_apf_dis[i])/10*apf_f[0];
            f_apf_total[1]=f_apf_total[1]+(effective_dis-env.global_apf_dis[i])/10*apf_f[1];
//            printf("angle_start:%d\tangle_end:%d\tangle_avg:%f\tdis:%d\n",env.global_angle_start[i],env.global_angle_end[i],angle_avg[number_obstacle],env.global_apf_dis[i]);
            //printf("x_apf_total:%f\ty_apf_total:%f\n",f_apf_total[0],f_apf_total[1]);
            //printf("apf_f_x:%f\tapf_f_y:%f\n\n",apf_f[0],apf_f[1]);
            ///////////////////f_total to unit vector for image_f cal//
            f_total[0]=(k_r[0]*apf_f[0]+k_g[0]*f_g[0]);
            f_total[1]=(k_r[1]*apf_f[1]+k_g[1]*f_g[1]);

            total_f_apf_goal=sqrt(f_total[0]*f_total[0]+f_total[1]*f_total[1]);

            if(total_f_apf_goal!=0){
                f_total[0]=f_total[0]/total_f_apf_goal;
                f_total[1]=f_total[1]/total_f_apf_goal;
            }
            //printf("v_x_without_image:%f\tv_y_without_image:%f\n\n",f_total[0],f_total[1]);
            /////////////////cal_f_image for angle////////////////
            Velocity_angle=atan2(f_total[0],f_total[1])*rad2deg;
            f_r_angle=atan2(apf_f[0],apf_f[1])*rad2deg;


            if(f_r_angle<0){f_r_angle=f_r_angle+360;}
            if(Velocity_angle<0){Velocity_angle=Velocity_angle+360;}
            if(goal_angle<0){f_g_angle=goal_angle+360;}
            //printf("f_r_angle:%f\tf_g_angle:%f\tVelocity_angle:%f\n",f_r_angle,f_g_angle,Velocity_angle);

            //////////cal image f///////////////
            if(goal_dis!=0){
                e_image_r[0]=(f_g[1]*f_total[2]-f_g[2]*f_total[1])/goal_dis*total_f_apf_goal*sin(fabs(goal_angle-Velocity_angle)*deg2rad);
                e_image_r[1]=(f_g[2]*f_total[0]-f_g[0]*f_total[2])/goal_dis*total_f_apf_goal*sin(fabs(goal_angle-Velocity_angle)*deg2rad);
                e_image_r[2]=(f_g[0]*f_total[1]-f_g[1]*f_total[0])/goal_dis*total_f_apf_goal*sin(fabs(goal_angle-Velocity_angle)*deg2rad);
            }

            f_image_r[0]=e_image_r[1]*apf_f[2]-e_image_r[2]*apf_f[1];
            f_image_r[1]=e_image_r[2]*apf_f[0]-e_image_r[0]*apf_f[2];
            f_image_r[2]=e_image_r[0]*apf_f[1]-e_image_r[1]*apf_f[0];

            //printf("f_image_x:%f\tf_image_y:%f\n",f_image_r[0],f_image_r[1]);

            if(f_r_angle<180){f_r_angle=f_r_angle+180;}//need to get ob_angle
            else{f_r_angle=f_r_angle-180;}

            double decay_time_image=93*deg2rad;//need to test

            k_image[0]=2/(1+exp(fabs(f_r_angle-Velocity_angle)*deg2rad/decay_time_image));
            k_image[1]=2/(1+exp(fabs(f_r_angle-Velocity_angle)*deg2rad/decay_time_image));

            //printf("k_image_x:%f\tKimage_y:%f\n",k_image[0],k_image[1]);
            f_image_total[0]=f_image_total[0]+k_image[0]*f_image_r[0];
            f_image_total[1]=f_image_total[0]+k_image[1]*f_image_r[1];
            //printf("f_image_x:%f\tf_image_y:%f\n\n",f_image_total[0],f_image_total[1]);
            //////////////////end/////////////////////

        }
    }

    env.global_angle_end.clear();
    env.global_angle_start.clear();
    env.global_apf_dis.clear();

    std::vector<int>().swap(env.global_angle_end);
    std::vector<int>().swap(env.global_angle_start);
    std::vector<int>().swap(env.global_apf_dis);

    angle_avg.clear();
    std::vector<double>().swap(angle_avg);
//    printf("x_image:%f\ty_image:%f\n",k_image[0]*f_image_r[0],k_image[1]*f_image_r[1]);

    total_f_apf=sqrt(f_apf_total[0]*f_apf_total[0]+f_apf_total[1]*f_apf_total[1]);
    if(total_f_apf!=0){
        f_apf_total[0]=f_apf_total[0]/total_f_apf;
        f_apf_total[1]=f_apf_total[1]/total_f_apf;
    }
    if(goal_dis<=1.5){
        k_g[0]=1;
        k_g[1]=1;
        k_r[0]=0;
        k_r[1]=0;
        k_image_r[0]=0;
        k_image_r[1]=0;
    }
//    /////////////////
    f_total[0]=k_r[0]*f_apf_total[0]+k_g[0]*f_g[0]+k_image_r[0]*f_image_total[0];
    f_total[1]=k_r[1]*f_apf_total[1]+k_g[1]*f_g[1]+k_image_r[1]*f_image_total[1];
    f_r_angle=atan2(f_apf_total[1],f_apf_total[0])*rad2deg;
    printf("f_r_angle:%f\n",f_r_angle);
    printf("k_r-x:%f,k_r-y:%f\tk_g-x:%f,k_g-y:%f\n",k_r[0],k_r[1],k_g[0],k_g[1]);
    printf("ft_x:%f\tft_y:%f\n\n",f_total[0],f_total[1]);
    if(number_obstacle==0){
        f_r_angle=0;
    }
//    ////////////real_state/////
    env.home[r_number].v_x = f_total[0];
    env.home[r_number].v_y = f_total[1];
    env.home[r_number].v_yaw = f_r_angle;
//    printf("v_x=%lf\n",env.home[r_number].v_x);
//    printf("=============end_f=======\n");

//    ////////////test_state//////////
////    if(goal_dis<=0.75){
////        env.home[r_number].v_x = 0;
////        env.home[r_number].v_y = 0;
////        env.home[r_number].v_yaw = 0;
////        printf("=====get_goal=====\n");
////    }else{
////        env.home[r_number].v_x = f_total[0];
////        env.home[r_number].v_y = f_total[1];
////        env.home[r_number].v_yaw = goal_angle;
////        printf("=============end_f=======\n");
////    }
//    ////////////////////test_mode_end////////////
//    angle_avg.clear();
//    std::vector<double>().swap(angle_avg);
//    number_obstacle=0;
}

double FIRA_pathplan_class::vecAngle(Vector2d a,Vector2d b){

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
void FIRA_pathplan_class::strategy_Support_CatchBallState(int r_number){

    double ball_dis = env.home[r_number].ball.distance;
    double ball_angle =env.home[r_number].ball.angle;
    double r_robot =1;
    if(fabs(ball_angle)<=0.00001)ball_angle=0.00001;//angle too small equal 0.0001
    double c_ball_dis = sqrt((ball_dis*ball_dis)+(r_robot*r_robot) - (2*ball_dis*r_robot*cos(ball_angle*deg2rad)));
    int sign=1; // ball is on left hand side
    if(ball_angle<0)sign=(-1); //ball is on right hand side
    double c_ball_angle = sign*(180-rad2deg*acos(((c_ball_dis*c_ball_dis)+(r_robot*r_robot)-(ball_dis*ball_dis))/(2*c_ball_dis*r_robot))/*cos AandC theta*/);//robot_head to ball dis
    if(ball_angle==0.00001)c_ball_angle=ball_angle;
    double goal_angle = env.home[r_number].op_goal.angle;//face target goal. left hand 0~-180, right hand 0~180
    double distance_br = env.home[r_number].ball.distance-0.2;
    if(distance_br<0){
        distance_br=0;
    }
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double left_dis;
    double left_theta;
    double right_dis;
    double right_theta;
    double angle_left_bg;
    double angle_right_bg;
    double left_bg_dis;
    double right_bg_dis;
    double left_omega;
    double right_omega;

    if(mTeam == Team_Blue){//attack goal is left,goal.dis is left // to decide Y is + or -
            left_dis = distance_dr;
            left_theta = angle_dr;
            right_dis = op_distance_dr;
            right_theta = op_angle_dr;
     }else if(mTeam == Team_Yellow){
            left_dis = op_distance_dr;
            left_theta = op_angle_dr;
            right_dis = distance_dr;
            right_theta = angle_dr;
    }
        angle_left_bg=left_theta-ball_angle;
        if(angle_left_bg>180){
            angle_left_bg=-360+angle_left_bg;
        }else if(angle_left_bg<-180){
            angle_left_bg=360+angle_left_bg;
        }
        angle_left_bg=fabs(angle_left_bg);
//        printf("angle_left_bg=%f\n",angle_left_bg);
        left_bg_dis=sqrt(left_dis*left_dis+ball_dis*ball_dis-2*left_dis*ball_dis*cos(angle_left_bg*deg2rad));
//        printf("left_bg_dis=%f\n",left_bg_dis);
        left_omega=acos((ball_dis*ball_dis+left_bg_dis*left_bg_dis-left_dis*left_dis)/(2*ball_dis*left_bg_dis))*rad2deg;
//        printf("left_omega=%f\n",left_omega);
        //------------------------------------------------------------------------
        angle_right_bg=right_theta-ball_angle;
        if(angle_right_bg>180){
            angle_right_bg=-360+angle_right_bg;
        }else if(angle_right_bg<-180){
            angle_right_bg=360+angle_right_bg;
        }
        angle_right_bg=fabs(angle_right_bg);
//        printf("angle_right_bg=%f\n",angle_right_bg);
        right_bg_dis=sqrt(right_dis*right_dis+ball_dis*ball_dis-2*right_dis*ball_dis*cos(angle_right_bg*deg2rad));
//        printf("right_bg_dis=%f\n",right_bg_dis);
        right_omega=acos((ball_dis*ball_dis+right_bg_dis*right_bg_dis-right_dis*right_dis)/(2*ball_dis*right_bg_dis))*rad2deg;
//        printf("right_omega=%f\n",right_omega);

        if(left_omega<right_omega){
            goal_angle=left_theta;
        }else{
            goal_angle=right_theta;
        }
//    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXball_dis=%f\n",distance_br);
    double ball_x = -c_ball_dis*sin(c_ball_angle*deg2rad);// x speed;**********************
    double ball_y = c_ball_dis*cos(c_ball_angle*deg2rad);// y speed;**********************
    double alpha = goal_angle - c_ball_angle;//-
    double beta = atan2( beta_const , c_ball_dis) * rad2deg;
    Vector2d vectorbr(ball_x, ball_y);
    if(beta < alpha){
        alpha = beta;
    }else if(alpha < -beta){
        alpha = -beta;
    }
    double rotAngle = -alpha;
    Rotation2Dd rot( rotAngle * deg2rad);
    Vector2d vectornt = rot * vectorbr;
    env.home[r_number].v_y =vectornt(1);
    env.home[r_number].v_x =vectornt(0);//vectornt(0);
    env.home[r_number].v_yaw =angle_br;//goal_angle;//*****************************face goal forever
//    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXball_dis=%f\n",ball_dis);
//    if(fabs(left_omega-right_omega)<2){
//        env.home[r_number].v_y =0;
//        env.home[r_number].v_x =0;//vectornt(0);
//    }
    double v_angle_br;
    v_angle_br=angle_br+90;
    if(v_angle_br>180){
        v_angle_br=-360+v_angle_br;
    }else if(v_angle_br<-180){
        v_angle_br=360+v_angle_br;
    }
    double vectorbr_x = distance_br * cos(v_angle_br* deg2rad);
    double vectorbr_y = distance_br * sin(v_angle_br* deg2rad);
//    printf("ball_dis=%f\n",ball_dis);
    if(ball_dis<1.2){
//        printf("worked\n");
        env.home[r_number].v_x = -vectorbr_x;
        env.home[r_number].v_y = -vectorbr_y;
    }else if((ball_dis<1.3)||(fabs(left_omega-right_omega)<2)){
            env.home[r_number].v_x = 0;
            env.home[r_number].v_y = 0;
     }
        if(fabs(angle_br)<10){
            env.home[r_number].v_yaw =0;
      }
     shoot = 0;

}

void FIRA_pathplan_class::strategy_Support_LostBallState(int r_number){
    printf("Defend (pathplan)\n");
    double angle_br =env.home[r_number].ball.angle;
    double distance_br =env.home[r_number].ball.distance;
    double angle_rg =env.home[r_number].goal.angle;
    double distance_rg =env.home[r_number].goal.distance;
    double angle_rog =env.home[r_number].op_goal.angle;
    double distance_rog =env.home[r_number].op_goal.distance;

    double t_angle_br=angle_br+90;
    double t_angle_rg=angle_rg+90;
    double t_angle_rog=angle_rog+90;

    if(t_angle_br>180){
        t_angle_br=(t_angle_br-360);
     }
    if(t_angle_rg>180){
        t_angle_rg=(t_angle_rg-360);
     }
    if(t_angle_rog>180){
        t_angle_rog=(t_angle_rog-360);
     }
    if(distance_rog>=distance_br){//defend goal dis > ball dis , move to defend goal
        printf("fabs(distance_rog-distance_br)=%f\n",fabs(distance_rog-distance_br));
              env.home[r_number].v_x =cos(t_angle_rog*deg2rad)*distance_rog;
              env.home[r_number].v_y =sin( t_angle_rog*deg2rad)*distance_rog;
              if(fabs(distance_rog-distance_br)<1){
                  env.home[r_number].v_x =0.5*cos(t_angle_rog*deg2rad)*distance_rog;
                  env.home[r_number].v_y =0.5*sin( t_angle_rog*deg2rad)*distance_rog;
                  if(fabs(distance_rog-distance_br)<0.5){
                      env.home[r_number].v_x =0.1*cos(t_angle_rog*deg2rad)*distance_rog;
                      env.home[r_number].v_y =0.1*sin( t_angle_rog*deg2rad)*distance_rog;
                      if(fabs(distance_rog-distance_br)<0.2){
                          env.home[r_number].v_x =0.01*cos(t_angle_rog*deg2rad)*distance_rog;
                          env.home[r_number].v_y =0.01*sin( t_angle_rog*deg2rad)*distance_rog;
                          if(fabs(distance_rog-distance_br)<0.05){
                              env.home[r_number].v_x =0;
                              env.home[r_number].v_y =0;
                          }
                      }
                  }
              }
    }else{//move to defend ball
        printf("fabs(distance_rog-distance_br)=%f\n",fabs(distance_rog-distance_br));
              env.home[r_number].v_x =cos(t_angle_br*deg2rad)*distance_br;
              env.home[r_number].v_y =sin(t_angle_br*deg2rad)*distance_br;
              if(fabs(distance_rog-distance_br)<1){
                  env.home[r_number].v_x =0.5*cos(t_angle_br*deg2rad)*distance_br;
                  env.home[r_number].v_y =0.5*sin(t_angle_br*deg2rad)*distance_br;
                  if(fabs(distance_rog-distance_br)<0.5){
                      env.home[r_number].v_x =0.1*cos(t_angle_br*deg2rad)*distance_br;
                      env.home[r_number].v_y =0.1*sin(t_angle_br*deg2rad)*distance_br;
                      if(fabs(distance_rog-distance_br)<0.2){
                          env.home[r_number].v_x =0.01*cos(t_angle_br*deg2rad)*distance_br;
                          env.home[r_number].v_y =0.01*sin(t_angle_br*deg2rad)*distance_br;
                          if(fabs(distance_rog-distance_br)<0.05){
                              env.home[r_number].v_x =0;
                              env.home[r_number].v_y =0;

                              }
                          }
                      }
                  }

    }

//               else
//               {
//                      env.home[r_number].v_x =cos(t_angle_br*deg2rad)*distance_br;
//                      env.home[r_number].v_y =sin(t_angle_br*deg2rad)*distance_br;
//               }
    env.home[r_number].v_yaw=angle_br;
    if(fabs(angle_br)<40){
        env.home[r_number].v_yaw=0.5*angle_br;
        if(fabs(angle_br)<20){
            env.home[r_number].v_yaw=0.1*angle_br;
            if(fabs(angle_br)<10){
                env.home[r_number].v_yaw=0.01*angle_br;
                if(fabs(angle_br)<5){
                    env.home[r_number].v_yaw=0.001*angle_br;
                    if(fabs(angle_br)<1){
                        env.home[r_number].v_yaw=0;
                    }
                }
            }
        }
    }
    shoot = 0;
}
void FIRA_pathplan_class::strategy_Support_Positioning(int r_number){
        double distance_br = env.home[r_number].ball.distance;
        double distance_dr = env.home[r_number].goal.distance;
        double op_distance_dr = env.home[r_number].op_goal.distance;
        double angle_br = env.home[r_number].ball.angle;
        double angle_dr = env.home[r_number].goal.angle;
        double op_angle_dr = env.home[r_number].op_goal.angle;
        double left_dis;
        double left_theta;
        double right_dis;
        double right_theta;
        double y_goal_theta;
        double vision_rx, vision_ry;
        double vision_rotation;
        double vision_bx, vision_by;
        if(mTeam == Team_Blue){//attack goal is left,goal.dis is left // to decide Y is + or -
                left_dis = distance_dr;
                left_theta = angle_dr;
                right_dis = op_distance_dr;
                right_theta = op_angle_dr-left_theta;
                if(right_theta>180){
                    right_theta=right_theta-360;
                }else if(right_theta<-180){
                    right_theta=right_theta+360;
                }
                y_goal_theta=(left_dis*left_dis+6.7*6.7-right_dis*right_dis)/(2*left_dis*6.7);
                y_goal_theta=acos(y_goal_theta);
                //printf("acos(y_goal_theta)*rad2deg=%f\n",y_goal_theta*rad2deg);
                vision_ry=left_dis*sin(y_goal_theta);
                vision_rx=-3.35+left_dis*cos(y_goal_theta);
                if(right_theta<0){
                    vision_ry=-vision_ry;
                }
                printf("left_dis=%f\n",left_dis);
                printf("right_dis=%f\n",right_dis);
         }else if(mTeam == Team_Yellow){
                left_dis = op_distance_dr;
                left_theta = op_angle_dr;
                right_dis = distance_dr;
                right_theta = angle_dr-left_theta;
                if(right_theta>180){
                    right_theta=right_theta-360;
                }else if(right_theta<-180){
                    right_theta=right_theta+360;
                }
                y_goal_theta=(left_dis*left_dis+6.7*6.7-right_dis*right_dis)/(2*left_dis*6.7);
                y_goal_theta=acos(y_goal_theta);
                //printf("acos(y_goal_theta)*rad2deg=%f\n",y_goal_theta*rad2deg);
                vision_ry=left_dis*sin(y_goal_theta);
                vision_rx=-3.35+left_dis*cos(y_goal_theta);
                if(right_theta<0){
                    vision_ry=-vision_ry;
                }
                printf("left_dis=%f\n",left_dis);
                printf("right_dis=%f\n",right_dis);
        }
        vision_rotation= atan2( 0-vision_ry,-3.35-vision_rx) * rad2deg;
        vision_rotation=vision_rotation-left_theta;
        if(vision_rotation>180){
            vision_rotation=-360+vision_rotation;
        }else if(vision_rotation<-180){
            vision_rotation=360+vision_rotation;
        }
        double b_rotation=angle_br+vision_rotation;
        if(b_rotation>180){
            b_rotation=-360+b_rotation;
        }else if(b_rotation<-180){
            b_rotation=360+b_rotation;
        }
        vision_bx = vision_rx+distance_br*cos(b_rotation*deg2rad);
        vision_by = vision_ry+distance_br*sin(b_rotation*deg2rad);
        // vision_rx,ry,rotation,bx,by
     // -------------------------------------------------------------------------------------------------------------------------------------
       double target_x=1,target_y=-1;
       double target_dis = sqrt((target_x-vision_rx)*(target_x-vision_rx)+(target_y-vision_ry)*(target_y-vision_ry));
       double vector_x = target_x - vision_rx;
       double vector_y =target_y- vision_ry;

       double phi_br = atan2( vector_y, vector_x) * rad2deg;
       double rb_angle = env.home[r_number].ball.angle;
       double r_basic_xy;
       r_basic_xy=vision_rotation-90;
       if(r_basic_xy<-180){
           r_basic_xy=360+r_basic_xy;
       }else if(r_basic_xy>180){
           r_basic_xy=-360+r_basic_xy;
       }//normalization

       double angle_xy_target=phi_br-r_basic_xy;
       if(angle_xy_target<-180){
           angle_xy_target=360+angle_xy_target;
       }else if(angle_xy_target>180){
           angle_xy_target=-360+angle_xy_target;
       }//normalization
       env.home[r_number].v_x =cos(angle_xy_target*deg2rad)*target_dis;
       env.home[r_number].v_y =sin(angle_xy_target*deg2rad)*target_dis;

       printf("r_basic_xy=%f\n",r_basic_xy);
       printf("angle_xy_target=%f\n",angle_xy_target);
       env.home[r_number].v_yaw=rb_angle;
       shoot = 0;
       printf("target_x=%f\n",target_x);
       printf("target_y=%f\n",target_y);
       printf("vision_rx=%f\n",vision_rx);
       printf("vision_ry=%f\n",vision_ry);
       printf("vision_rotation=%f\n",vision_rotation);
       printf("vision_bx=%f\n",vision_bx);
       printf("vision_by=%f\n",vision_by);
}

void FIRA_pathplan_class::strategy_Support_Test1(int r_number){
        double distance_br = env.home[r_number].ball.distance;
        double distance_dr = env.home[r_number].goal.distance;
        double op_distance_dr = env.home[r_number].op_goal.distance;
        double angle_br = env.home[r_number].ball.angle;
        double angle_dr = env.home[r_number].goal.angle;
        double op_angle_dr = env.home[r_number].op_goal.angle;
        double angle_goal_large_area = env.home[r_number].goal_large_area.angle;
        double distance_goal_large_area = env.home[r_number].goal_large_area.distance;
        double angle_opgoal_large_area = env.home[r_number].op_goal_large_area.angle;
        double distance_opgoal_large_area = env.home[r_number].op_goal_large_area.distance;
        printf("goal angle =%f, distance =%f\n",angle_goal_large_area,distance_goal_large_area);
        printf("opgoal angle =%f, op_distance =%f\n",angle_opgoal_large_area,distance_opgoal_large_area);


}
void FIRA_pathplan_class::strategy_Support_Test2(int r_number){


    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double x_speed = 0;
    double y_speed = 0;
    double yaw_speed = 0;

    double to_middle_line_dis = sin(op_angle_dr*deg2rad)*op_distance_dr; //distance to (goal && op_goal) line
    static int x_moving=0; // move left or move right plan, 0:default, 1:move to left, 2:move to right
    static int cross_field_count =0;
    double define_middle_line_dis =0.8; // define how far do you want to leave from middle_line
    double y_distance_dr=fabs(cos(angle_dr*deg2rad))*distance_dr; //robot head direction y vector distance_dr
    double y_distance_opdr=fabs(cos(op_angle_dr*deg2rad))*op_distance_dr; //robot head direction y vector distance_opdr
    double x_distance_opdr=fabs(sin(op_angle_dr*deg2rad))*op_distance_dr; //robot head direction x vector distance_opdr
    double y_distance_br=fabs(cos(angle_br*deg2rad))*distance_br; //robot head direction y vector distance_br
    double x_distance_br=fabs(sin(angle_br*deg2rad))*distance_br; //robot head direction x vector distance_br

    //#######################################################//
    //                                                       //
    //    angle transform
    //    1.inv_op_angle_dr = tail face op_angle_dr
    //    2.transform_angle_dr = angle_dr
    //                                                       //
    //#######################################################//

    double transform_angle_dr = env.home[r_number].goal.angle;
    double inv_op_angle_dr=op_angle_dr+180;// let tail be head
    if(inv_op_angle_dr>180){
        inv_op_angle_dr=inv_op_angle_dr-360;
    }

    if(inv_op_angle_dr>100&&transform_angle_dr<-100){//+and -
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }else if(inv_op_angle_dr<-100&&transform_angle_dr>100){
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }

    //#######################################################//
    //                                                       //
    //    giving speed
    //    1.v_yaw = (head to angle_dr) + (tail to op_angle_dr)
    //    2.v_x = move to define_middle_line_dis base on where you are
    //    3.v_y = y_distance_br*1.5-y_distance_opdr
    //    ## all detect number need to below 3.5 meters, or will not correct
    //                                                       //
    //#######################################################//

    //---------------------------------------yaw_speed--------------------------
    //yaw_speed = inv_op_angle_dr+transform_angle_dr; // (head to angle_dr) + (tail to op_angle_dr) = face forward

    //---------------------------------------x_speed---------------------------
    if((op_angle_dr<0&&angle_dr<0)&&(angle_br>=0)){//robot && ball left side, so ready to "move to right" side define_dis
        x_moving =2;
    }else if((op_angle_dr>=0&&angle_dr>=0)&&(angle_br<0)){//robot && ball right side, so ready to "move to left" side define_dis
        x_moving =1;
    }

    if(fabs(inv_op_angle_dr+angle_dr)<20){// if face forward correct within error interval, give x vector speed
        switch(x_moving){
            case 1://move to left
                printf("move to left\n");
                x_speed = -to_middle_line_dis-define_middle_line_dis;
                if(to_middle_line_dis<-define_middle_line_dis){
                    printf("x_speed = 0\n");
                    x_speed = 0;
                    if(x_distance_br<x_distance_opdr){//ball across field
                        cross_field_count++;
                        if(cross_field_count>3){
                            x_moving=2;
                            cross_field_count=0;
                        }else{
                            cross_field_count=0;
                        }
                    }
                }else{
                    printf("x have speed\n");
                }
            break;
            case 2://move to right
                printf("move to right\n");
                x_speed = -to_middle_line_dis+define_middle_line_dis;
                if(to_middle_line_dis>define_middle_line_dis){
                    printf("x_speed = 0\n");
                    x_speed = 0;
                    if(x_distance_br<x_distance_opdr){//ball across field
                        cross_field_count++;
                        if(cross_field_count>3){
                            x_moving=1;
                            cross_field_count=0;
                        }else{
                            cross_field_count=0;
                        }
                    }
                }else{
                    printf("x have speed\n");
                }
            break;
            default:
                if(op_angle_dr<0 && angle_dr<0){// left side, move to left side position
                    printf("move to left\n");
                    x_speed = -to_middle_line_dis-define_middle_line_dis;
                    if(to_middle_line_dis<-define_middle_line_dis){
                        printf("x_speed = 0\n");
                        x_speed = 0;
                        if(x_distance_br<x_distance_opdr){//ball across field
                            cross_field_count++;
                            if(cross_field_count>3){
                                x_moving=2;
                                cross_field_count=0;
                            }else{
                                cross_field_count=0;
                            }
                        }
                    }
                }else if(op_angle_dr>=0 && angle_dr>=0){// right side, move to right side position
                    printf("move to right\n");
                    x_speed = -to_middle_line_dis+define_middle_line_dis;
                    if(to_middle_line_dis>define_middle_line_dis){//almost to right position
                        printf("x_speed = 0\n");
                        x_speed = 0;
                        if(x_distance_br<x_distance_opdr){//ball across field
                            cross_field_count++;
                            if(cross_field_count>3){
                                x_moving=1;
                                cross_field_count=0;
                            }else{
                                cross_field_count=0;
                            }
                        }
                    }
                }
            break;

        }
        //------------------------y_speed-------------------------

        if(op_distance_dr<3){// if not across defend gate
            if(angle_br<70&&angle_br>-70){//if ball in front of robot
              y_speed = y_distance_br*2-y_distance_opdr; // give y speed
            }else{//ball at robot back
               if(x_moving==1){
                   printf("move to left edge\n");
                   strategy_MovetoOpGoalEdge2(r_number);
               }else if(x_moving==2){
                   printf("move to right edge\n");
                   strategy_MovetoOpGoalEdge1(r_number);
               }
            }

        }else{// across defend gate, move back
            printf("too far from defend gate\n");
            y_speed = 0;
            printf("x_speed = 0\n");
            if(angle_br<-70||angle_br>70){//if ball back of robot
              y_speed = y_distance_br*2-y_distance_opdr; // give y speed
            }
        }


    }else{
        x_speed =0;
        y_speed =0;
        yaw_speed = inv_op_angle_dr+transform_angle_dr;
    }

    env.home[r_number].v_yaw = yaw_speed*2;
    env.home[r_number].v_x = x_speed*speed_constant;
    env.home[r_number].v_y = y_speed*speed_constant;

    //#######################################################//
    //                                                       //
    //    lowest speed limit (if to low, set it to zero)
    //                                                       //
    //#######################################################//

    if(fabs(yaw_speed)<yaw_speed_limit){
       env.home[r_number].v_yaw= 0;
    }
    if((fabs(x_speed*speed_constant)<speed_limit)&&(fabs(y_speed*speed_constant)<speed_limit)){
       env.home[r_number].v_x = 0;
       env.home[r_number].v_y = 0;
    }
    shoot = 0;

//    printf("x_speed*speed_constant=%f\n",x_speed*speed_constant);
//    printf("y_speed*speed_constant=%f\n",y_speed*speed_constant);
//    printf("yaw_speed=%f\n",yaw_speed);

//    printf("FFFFf=%f\n",fabs(y_distance_br*2-y_distance_opdr));
    printf("y_distance_br=%f\n",y_distance_br);
    printf("y_distance_opdr=%f\n",y_distance_opdr);
    printf("y_distance_dr=%f\n",y_distance_dr);
    printf("x_distance_opdr=%f\n",x_distance_opdr);
    printf("x_distance_br=%f\n",x_distance_br);
    printf("to_middle_line_dis=%f\n",to_middle_line_dis);

    printf("op_angle_dr=%f\n",op_angle_dr);



}
void FIRA_pathplan_class::strategy_Support_Test3(int r_number){

//    double distance_br = env.home[r_number].ball.distance;
//    double distance_dr = env.home[r_number].goal.distance;
//    double op_distance_dr = env.home[r_number].op_goal.distance;
//    double angle_br = env.home[r_number].ball.angle;
//    double angle_dr = env.home[r_number].goal_edge.angle_1/2+env.home[r_number].goal_edge.angle_2/2;
//    double op_angle_dr = env.home[r_number].op_goal_edge.angle_1/2+env.home[r_number].op_goal_edge.angle_2/2;
//    double to_middle_line_dis = sin(op_angle_dr*deg2rad)*op_distance_dr;
//    static int x_moving=0;
//    double x_speed=0;
//    double define_middle_line_dis =0.5;
//    double y_distance_dr=fabs(cos(angle_dr*deg2rad))*distance_dr;
//    double y_distance_opdr=fabs(cos(op_angle_dr*deg2rad))*op_distance_dr;
//    double x_distance_opdr=fabs(sin(op_angle_dr*deg2rad))*op_distance_dr;
//    double y_distance_br=fabs(cos(angle_br*deg2rad))*distance_br;
//    double x_distance_br=fabs(sin(angle_br*deg2rad))*distance_br;

//    //--------------------------------------------------------------------face foward
//            double transform_angle_dr = env.home[r_number].goal.angle;
//            double inv_op_angle_dr=op_angle_dr+180;// let tail be head
//            if(inv_op_angle_dr>180){
//                inv_op_angle_dr=inv_op_angle_dr-360;
//            }

//            if(inv_op_angle_dr>100&&transform_angle_dr<-100){//+and -
//                if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
//                    inv_op_angle_dr=0;
//                }else{
//                    transform_angle_dr=0;
//                }
//            }else if(inv_op_angle_dr<-100&&transform_angle_dr>100){
//                if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
//                    inv_op_angle_dr=0;
//                }else{
//                    transform_angle_dr=0;
//                }
//            }


//            env.home[r_number].v_yaw=inv_op_angle_dr+transform_angle_dr;
//            if(fabs(inv_op_angle_dr+transform_angle_dr)<1){
//               env.home[r_number].v_yaw=0;
//            }


//    if((op_angle_dr<0&&angle_dr<0)&&(angle_br>=0)){//robot && ball left side
////        printf("left side\n");
//        x_moving =2;
//    }else if((op_angle_dr>=0&&angle_dr>=0)&&(angle_br<0)){//robot && ball right side
////        printf("right side\n");
//        x_moving =1;
//    }

//    if(fabs(inv_op_angle_dr+angle_dr)<20){//if face forward
//        switch(x_moving){
//            case 1://move to left
//                printf("move to left\n");
//                x_speed = -to_middle_line_dis-define_middle_line_dis;
//                env.home[r_number].v_x =x_speed*2;
//                if(fabs(x_speed)<0.1){
//                    env.home[r_number].v_x = 0;
//                    if(x_distance_br<x_distance_opdr){//ball across field
//                        x_moving=2;
//                    }
//                }
//            break;
//            case 2://move to right
//                printf("move to right\n");
//                x_speed = -to_middle_line_dis+define_middle_line_dis;
//                env.home[r_number].v_x =x_speed*2;
//                if(fabs(x_speed)<0.1){
//                    env.home[r_number].v_x = 0;
//                    if(x_distance_br<x_distance_opdr){//ball across field
//                        x_moving=1;
//                    }
//                }
//            break;
//            default:
//                if(op_angle_dr<0 && angle_dr<0){// left side, move to left side position
//                    printf("move to left\n");
//                    x_speed = -to_middle_line_dis-define_middle_line_dis;
//                    env.home[r_number].v_x =x_speed*2;
//                    if(fabs(x_speed)<0.1){
//                        env.home[r_number].v_x = 0;
//                        if(x_distance_br<x_distance_opdr){//ball across field
//                            x_moving=2;
//                        }
//                    }
//                }else if(op_angle_dr>=0 && angle_dr>=0){// right side, move to right side position
//                    printf("move to right\n");
//                    x_speed = -to_middle_line_dis+define_middle_line_dis;
//                    env.home[r_number].v_x =x_speed*2;
//                    if(fabs(x_speed)<0.1){//almost to right position
//                        env.home[r_number].v_x = 0;
//                        if(x_distance_br<x_distance_opdr){//ball across field
//                            x_moving=1;
//                        }
//                    }
//                }
//            break;

//        }
//        if(y_distance_opdr<3&& op_distance_dr<3.4){// if not across defend gate
//            if(cos(angle_br*deg2rad)>=0){//ball in front of robot
//              env.home[r_number].v_y=y_distance_br*1.5-y_distance_opdr;
//              if(y_distance_br>1.5 && y_distance_opdr>=2.5){//farest distance
//                  printf("farest distance\n");
//                  env.home[r_number].v_y=0;
//              }else if(fabs(y_distance_br*2-y_distance_opdr)<0.2){
//                  printf("moving speed too small\n");
//                  env.home[r_number].v_y=0;
//              }else{
//                  printf("ball in front of robot :%f\n",0);
//              }
//            }else{//ball at robot back
//               printf("back\n");
//              env.home[r_number].v_y=-y_distance_opdr;
//            }

//        }else{// across defend gate, move back
//            printf("too far from defend gate\n");
//            if(op_distance_dr>=3.4){
//              env.home[r_number].v_y=-op_distance_dr+3;
//            }else if(y_distance_opdr>=3){
//              env.home[r_number].v_y=-y_distance_opdr+3;
//            }
//        }


//    }else{
//        env.home[r_number].v_x =0;
//        env.home[r_number].v_y =0;
//    }
//    printf("FFFFf=%f\n",fabs(y_distance_br*2-y_distance_opdr));
//    printf("y_distance_br=%f\n",y_distance_br);
//    printf("y_distance_opdr=%f\n",y_distance_opdr);
//    printf("y_distance_dr=%f\n",y_distance_dr);
//    printf("x_distance_opdr=%f\n",x_distance_opdr);
//    printf("x_distance_br=%f\n",x_distance_br);
    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double v_rotation= 0;// to give robot head direction x & y speed to op_angle_dr
    double x_speed = 0;
    double y_speed = 0;
    double yaw_speed = 0;

    //#######################################################//
    //                                                       //
    //    angle transform
    //    1.inv_op_angle_dr = tail face op_angle_dr
    //    2.transform_angle_dr = angle_dr
    //    3.v_rotation = to give robot head direction x & y speed to op_angle_dr
    //                                                       //
    //#######################################################//

    double transform_angle_dr = env.home[r_number].goal.angle;
    double inv_op_angle_dr=op_angle_dr+180;// let tail be head
    if(inv_op_angle_dr>180){
        inv_op_angle_dr=inv_op_angle_dr-360;
    }

    if(inv_op_angle_dr>100&&transform_angle_dr<-100){//+and -
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }else if(inv_op_angle_dr<-100&&transform_angle_dr>100){
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }

    //#######################################################//
    //                                                       //
    //    giving speed
    //    1.v_yaw = (head to angle_dr) + (tail to op_angle_dr)
    //    2.v_x =
    //    3.v_y =
    //                                                       //
    //#######################################################//

    //-----------------------yaw_speed-------------------
    yaw_speed =inv_op_angle_dr+transform_angle_dr;

    //-----------------------x_speed---------------------
    //-----------------------y_speed---------------------

    v_rotation=env.home[r_number].goal_edge.angle_2+90;
    if(v_rotation>180){
        v_rotation=-360+v_rotation;
    }
    if(v_rotation<-180){
        v_rotation=360+v_rotation;
    }
    double v_rotation2=env.home[r_number].op_goal_edge.angle_1+90;
    if(v_rotation2>180){
        v_rotation2=-360+v_rotation2;
    }
    if(v_rotation2<-180){
        v_rotation2=360+v_rotation2;
    }
    x_speed = cos(v_rotation*deg2rad)*1+cos(v_rotation2*deg2rad)*op_distance_dr;
    y_speed = sin(v_rotation*deg2rad)*1+sin(v_rotation2*deg2rad)*op_distance_dr;


    env.home[r_number].v_yaw = yaw_speed;
    env.home[r_number].v_x = x_speed*speed_constant;
    env.home[r_number].v_y = y_speed*speed_constant;

    //#######################################################//
    //                                                       //
    //    lowest speed limit (if to low, set it to zero)
    //                                                       //
    //#######################################################//

    if(fabs(yaw_speed)<yaw_speed_limit){
       env.home[r_number].v_yaw= 0;
    }
    if((fabs(x_speed*speed_constant)<speed_limit)&&(fabs(y_speed*speed_constant)<speed_limit)){
       env.home[r_number].v_x = 0;
       env.home[r_number].v_y = 0;
    }
    shoot = 0;


}
void FIRA_pathplan_class::strategy_MovetoYellowGate(int r_number){

    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double v_rotation=0;// to give robot head direction x & y speed to op_angle_dr
    double x_speed = 0;
    double y_speed = 0;
    double yaw_speed = 0;

    //#######################################################//
    //                                                       //
    //    angle transform
    //    1.inv_op_angle_dr = tail face op_angle_dr
    //    2.transform_angle_dr = angle_dr
    //    3.v_rotation = to give robot head direction x & y speed to op_angle_dr
    //                                                       //
    //#######################################################//

    double transform_angle_dr = env.home[r_number].goal.angle;
    double inv_op_angle_dr=op_angle_dr+180;// let tail be head
    if(inv_op_angle_dr>180){
        inv_op_angle_dr=inv_op_angle_dr-360;
    }

    if(inv_op_angle_dr>100&&transform_angle_dr<-100){//+and -
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }else if(inv_op_angle_dr<-100&&transform_angle_dr>100){
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }

    //#######################################################//
    //                                                       //
    //    giving speed
    //    1.v_yaw = (head to angle_dr) + (tail to op_angle_dr)
    //    2.v_x =
    //    3.v_y =
    //                                                       //
    //#######################################################//

    //----------------------------yaw_speed----------------------

    yaw_speed =inv_op_angle_dr+transform_angle_dr;

    //-----------------------x_speed---------------------
    //-----------------------y_speed---------------------

    if(mTeam == Team_Blue){// goal is yellow
        v_rotation=angle_dr+90;
        if(v_rotation>180){
            v_rotation=-360+v_rotation;
        }
        if(v_rotation<-180){
            v_rotation=360+v_rotation;
        }
        x_speed = cos(v_rotation*deg2rad)*distance_dr;
        y_speed = sin(v_rotation*deg2rad)*distance_dr;

    }else if(mTeam == Team_Yellow){ //opgoal is yellow

        v_rotation=op_angle_dr+90;
        if(v_rotation>180){
            v_rotation=-360+v_rotation;
        }
        if(v_rotation<-180){
            v_rotation=360+v_rotation;
        }
        x_speed = cos(v_rotation*deg2rad)*op_distance_dr;
        y_speed = sin(v_rotation*deg2rad)*op_distance_dr;
    }

    env.home[r_number].v_yaw = yaw_speed;
    env.home[r_number].v_x = x_speed*speed_constant;
    env.home[r_number].v_y = y_speed*speed_constant;

    //#######################################################//
    //                                                       //
    //    lowest speed limit (if to low, set it to zero)
    //                                                       //
    //#######################################################//

    if(fabs(yaw_speed)<yaw_speed_limit){
       env.home[r_number].v_yaw= 0;
    }
    if((fabs(x_speed*speed_constant)<speed_limit)&&(fabs(y_speed*speed_constant)<speed_limit)){
       env.home[r_number].v_x = 0;
       env.home[r_number].v_y = 0;
    }
    shoot = 0;

//    printf("x_speed*speed_constant=%f\n",x_speed*speed_constant);
//    printf("y_speed*speed_constant=%f\n",y_speed*speed_constant);
//    printf("yaw_speed=%f\n",yaw_speed);

}
void FIRA_pathplan_class::strategy_MovetoBlueGate(int r_number){

    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double v_rotation= 0;// to give robot head direction x & y speed to op_angle_dr
    double x_speed = 0;
    double y_speed = 0;
    double yaw_speed = 0;

    //#######################################################//
    //                                                       //
    //    angle transform
    //    1.inv_op_angle_dr = tail face op_angle_dr
    //    2.transform_angle_dr = angle_dr
    //    3.v_rotation = to give robot head direction x & y speed to op_angle_dr
    //                                                       //
    //#######################################################//

    double transform_angle_dr = env.home[r_number].goal.angle;
    double inv_op_angle_dr=op_angle_dr+180;// let tail be head
    if(inv_op_angle_dr>180){
        inv_op_angle_dr=inv_op_angle_dr-360;
    }

    if(inv_op_angle_dr>100&&transform_angle_dr<-100){//+and -
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }else if(inv_op_angle_dr<-100&&transform_angle_dr>100){
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }

    //#######################################################//
    //                                                       //
    //    giving speed
    //    1.v_yaw = (head to angle_dr) + (tail to op_angle_dr)
    //    2.v_x =
    //    3.v_y =
    //                                                       //
    //#######################################################//

    //-----------------------yaw_speed-------------------
    yaw_speed =inv_op_angle_dr+transform_angle_dr;

    //-----------------------x_speed---------------------
    //-----------------------y_speed---------------------

    if(mTeam == Team_Blue){// goal is yellow,op is blue
        v_rotation=op_angle_dr+90;
        if(v_rotation>180){
            v_rotation=-360+v_rotation;
        }
        if(v_rotation<-180){
            v_rotation=360+v_rotation;
        }
        x_speed = cos(v_rotation*deg2rad)*op_distance_dr;
        y_speed = sin(v_rotation*deg2rad)*op_distance_dr;

    }else if(mTeam == Team_Yellow){ //opgoal is yellow, goal is blue

        v_rotation=angle_dr+90;
        if(v_rotation>180){
            v_rotation=-360+v_rotation;
        }
        if(v_rotation<-180){
            v_rotation=360+v_rotation;
        }
        x_speed = cos(v_rotation*deg2rad)*distance_dr;
        y_speed = sin(v_rotation*deg2rad)*distance_dr;
    }

    env.home[r_number].v_yaw = yaw_speed;
    env.home[r_number].v_x = x_speed*speed_constant;
    env.home[r_number].v_y = y_speed*speed_constant;

    //#######################################################//
    //                                                       //
    //    lowest speed limit (if to low, set it to zero)
    //                                                       //
    //#######################################################//

    if(fabs(yaw_speed)<yaw_speed_limit){
       env.home[r_number].v_yaw= 0;
    }
    if((fabs(x_speed*speed_constant)<speed_limit)&&(fabs(y_speed*speed_constant)<speed_limit)){
       env.home[r_number].v_x = 0;
       env.home[r_number].v_y = 0;
    }
    shoot = 0;

//    printf("x_speed*speed_constant=%f\n",x_speed*speed_constant);
//    printf("y_speed*speed_constant=%f\n",y_speed*speed_constant);
//    printf("yaw_speed=%f\n",yaw_speed);

}

void FIRA_pathplan_class::strategy_LeaveBall(int r_number){

    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double x_speed = 0;
    double y_speed = 0;
    double yaw_speed = 0;

    //#######################################################//
    //                                                       //
    //    angle transform
    //    1.inv_op_angle_dr = tail face op_angle_dr
    //    2.transform_angle_dr = angle_dr
    //    3.transform_angle_br = to give robot head direction x & y speed to angle_br
    //                                                       //
    //#######################################################//

    double transform_angle_dr = env.home[r_number].goal.angle;
    double inv_op_angle_dr=op_angle_dr+180;// let tail be head
    if(inv_op_angle_dr>180){
        inv_op_angle_dr=inv_op_angle_dr-360;
    }

    if(inv_op_angle_dr>100&&transform_angle_dr<-100){//+and -
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }else if(inv_op_angle_dr<-100&&transform_angle_dr>100){
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }

    double transform_angle_br=angle_br+90;
    if(transform_angle_br>180){
        transform_angle_br=transform_angle_br-360;
    }else if(transform_angle_br<-180){
        transform_angle_br=transform_angle_br+360;
    }

    double transform_angle_opdr=op_angle_dr+90;
    if(transform_angle_opdr>180){
        transform_angle_opdr=transform_angle_opdr-360;
    }else if(transform_angle_opdr<-180){
        transform_angle_opdr=transform_angle_opdr+360;
    }

    //#######################################################//
    //                                                       //
    //    giving speed
    //    1.v_yaw = (head to angle_dr) + (tail to op_angle_dr)
    //    2.v_x =
    //    3.v_y =
    //                                                       //
    //#######################################################//

    yaw_speed = inv_op_angle_dr+transform_angle_dr;
    printf("yaw_speed=%f\n",yaw_speed);

    if(fabs(inv_op_angle_dr+transform_angle_dr)<5){//robot face foward
        if((angle_br<-120||angle_br>120)&& distance_br<1){//if ball right behind robot
            if(angle_br<-120){//ball at right side, go to left
                x_speed = -1;
                y_speed = -sin(transform_angle_br*deg2rad)+sin(transform_angle_opdr*deg2rad);// leave ball + to opdr
            }else if(angle_br>120){//ball at left side, go to right
                x_speed = 1;
                y_speed = -sin(transform_angle_br*deg2rad)+sin(transform_angle_opdr*deg2rad);// leave ball + to opdr
            }
        }else{// ball is beside or infront robot
            x_speed = -cos(transform_angle_br*deg2rad)+cos(transform_angle_opdr*deg2rad);// leave ball + to opdr
            y_speed = -sin(transform_angle_br*deg2rad)+sin(transform_angle_opdr*deg2rad);// leave ball + to opdr
        }
    }else{//robot not face foward
        if(distance_br<1){
            x_speed = -cos(transform_angle_br*deg2rad)*2;// distance too small, leave ball
            y_speed = -sin(transform_angle_br*deg2rad)*2;// distance too small, leave ball
        }else{
            x_speed = 0;// distance ok, stop and turn forward first
            y_speed = 0;// distance ok, stop and turn forward first
        }
    }

    env.home[r_number].v_yaw = yaw_speed;
    env.home[r_number].v_x = x_speed*speed_constant;
    env.home[r_number].v_y = y_speed*speed_constant;

    //#######################################################//
    //                                                       //
    //    lowest speed limit (if to low, set it to zero)
    //                                                       //
    //#######################################################//

    if(fabs(yaw_speed)<yaw_speed_limit){
       env.home[r_number].v_yaw= 0;
    }
    if((fabs(x_speed*speed_constant)<speed_limit)&&(fabs(y_speed*speed_constant)<speed_limit)){
       env.home[r_number].v_x = 0;
       env.home[r_number].v_y = 0;
    }
    shoot = 0;

//    printf("x_speed*speed_constant=%f\n",x_speed*speed_constant);
//    printf("y_speed*speed_constant=%f\n",y_speed*speed_constant);
//    printf("yaw_speed=%f\n",yaw_speed);
//    printf("inv_op_angle_dr=%f\n",inv_op_angle_dr);
//    printf("transform_angle_dr=%f\n",transform_angle_dr);

}
void FIRA_pathplan_class::strategy_LeaveLimitArea(int r_number){

    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double x_speed = 0;
    double y_speed = 0;
    double yaw_speed = 0;

    //#######################################################//
    //                                                       //
    //    angle transform
    //    1.inv_op_angle_dr = tail face op_angle_dr
    //    2.transform_angle_dr = angle_dr
    //    3.transform_op_angle_dr = to give robot head direction x & y speed to op_angle_dr
    //                                                       //
    //#######################################################//

    double transform_angle_dr = env.home[r_number].goal.angle;
    double inv_op_angle_dr=op_angle_dr+180;// let tail be head
    if(inv_op_angle_dr>180){
        inv_op_angle_dr=inv_op_angle_dr-360;
    }

    if(inv_op_angle_dr>100&&transform_angle_dr<-100){//+and -
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }else if(inv_op_angle_dr<-100&&transform_angle_dr>100){
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }

    double transform_op_angle_dr=op_angle_dr+90;
    if(transform_op_angle_dr>180){
        transform_op_angle_dr=transform_op_angle_dr-360;
    }else if(transform_op_angle_dr<-180){
        transform_op_angle_dr=transform_op_angle_dr+360;
    }

    //#######################################################//
    //                                                       //
    //    giving speed
    //    1.v_yaw = (head to angle_dr) + (tail to op_angle_dr)
    //    2.v_x =
    //    3.v_y =
    //                                                       //
    //#######################################################//

    yaw_speed = inv_op_angle_dr+transform_angle_dr;
    if(op_distance_dr<1.9){// too inside limit area
        x_speed = -cos(transform_op_angle_dr*deg2rad)*(2-op_distance_dr);// if op_distance_dr less than 1.35, move out
        y_speed = -sin(transform_op_angle_dr*deg2rad)*(2-op_distance_dr);
    }else if(op_distance_dr>=1.9 && op_distance_dr <2){ // at edge of limit area
        x_speed = 0;
        y_speed = 0;
        if(angle_br<70&&angle_br>-70){
            x_speed = 0;
            y_speed = 3.5-op_distance_dr;
        }
    }

    env.home[r_number].v_yaw = yaw_speed;
    env.home[r_number].v_x = x_speed*speed_constant;
    env.home[r_number].v_y = y_speed*speed_constant;

    //#######################################################//
    //                                                       //
    //    lowest speed limit (if to low, set it to zero)
    //                                                       //
    //#######################################################//

    if(fabs(yaw_speed)<yaw_speed_limit){
       env.home[r_number].v_yaw= 0;
    }
    if((fabs(x_speed*speed_constant)<speed_limit)&&(fabs(y_speed*speed_constant)<speed_limit)){
       env.home[r_number].v_x = 0;
       env.home[r_number].v_y = 0;
    }
    shoot = 0;

//    printf("x_speed*speed_constant=%f\n",x_speed*speed_constant);
//    printf("y_speed*speed_constant=%f\n",y_speed*speed_constant);
//    printf("yaw_speed=%f\n",yaw_speed);

}

void FIRA_pathplan_class::strategy_LeftRightMove(int r_number){

    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double transform_angle_dr = angle_dr;
    double x_speed = 0;
    double y_speed = 0;
    double yaw_speed = 0;

    //#######################################################//
    //                                                       //
    //    angle transform
    //    1.inv_op_angle_dr = tail face op_angle_dr
    //    2.transform_angle_dr = angle_dr
    //    3.transform_angle_br = to give robot head direction x & y speed
    //                                                       //
    //#######################################################//

    double inv_op_angle_dr=op_angle_dr+180;// let tail be head
    if(inv_op_angle_dr>180){
        inv_op_angle_dr=inv_op_angle_dr-360;
    }

    if(inv_op_angle_dr>100&&transform_angle_dr<-100){//+and -
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }else if(inv_op_angle_dr<-100&&transform_angle_dr>100){
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }

    double transform_angle_br=angle_br;
    transform_angle_br=transform_angle_br+90;
    if(transform_angle_br>180){
        transform_angle_br=transform_angle_br-360;
    }else if(transform_angle_br<-180){
        transform_angle_br=transform_angle_br+360;
    }


    //#######################################################//
    //                                                       //
    //    giving speed
    //    1.v_yaw = (head to angle_dr) + (tail to op_angle_dr)
    //    2.v_x = robot to ball x vector speed
    //    3.v_y =0
    //                                                       //
    //#######################################################//

    yaw_speed = inv_op_angle_dr+transform_angle_dr; // (head to angle_dr) + (tail to op_angle_dr) = face forward

    if(fabs(inv_op_angle_dr+transform_angle_dr)<20){// if face forward correct within error interval, give x vector speed
       x_speed = -cos(transform_angle_br*deg2rad)*distance_br;
    }else{
       x_speed = 0;// if not face correct, give v_x=0, until you turn to right angle
    }

    env.home[r_number].v_yaw = yaw_speed;
    env.home[r_number].v_x = x_speed*speed_constant;
    env.home[r_number].v_y = y_speed*speed_constant;

    //#######################################################//
    //                                                       //
    //    lowest speed limit (if to low, set it to zero)
    //                                                       //
    //#######################################################//

    if(fabs(yaw_speed)<yaw_speed_limit){
       env.home[r_number].v_yaw= 0;
    }
    if((fabs(x_speed*speed_constant)<speed_limit)&&(fabs(y_speed*speed_constant)<speed_limit)){
       env.home[r_number].v_x = 0;
       env.home[r_number].v_y = 0;
    }
    shoot = 0;
//    printf("x_speed=%f\n",x_speed);
//    printf("y_speed=%f\n",y_speed);
//    printf("yaw_speed=%f\n",yaw_speed);

}

void FIRA_pathplan_class::strategy_invLeftRightMove(int r_number){

    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double x_speed = 0;
    double y_speed = 0;
    double yaw_speed = 0;

    double to_middle_line_dis = sin(op_angle_dr*deg2rad)*op_distance_dr; //distance to (goal && op_goal) line
    static int x_moving=0; // move left or move right plan, 0:default, 1:move to left, 2:move to right
    double define_middle_line_dis =0.6; // define how far do you want to leave from middle_line
    double y_distance_dr=fabs(cos(angle_dr*deg2rad))*distance_dr; //robot head direction y vector distance_dr
    double y_distance_opdr=fabs(cos(op_angle_dr*deg2rad))*op_distance_dr; //robot head direction y vector distance_opdr
    double x_distance_opdr=fabs(sin(op_angle_dr*deg2rad))*op_distance_dr; //robot head direction x vector distance_opdr
    double y_distance_br=fabs(cos(angle_br*deg2rad))*distance_br; //robot head direction y vector distance_br
    double x_distance_br=fabs(sin(angle_br*deg2rad))*distance_br; //robot head direction x vector distance_br

    //#######################################################//
    //                                                       //
    //    angle transform
    //    1.inv_op_angle_dr = tail face op_angle_dr
    //    2.transform_angle_dr = angle_dr
    //                                                       //
    //#######################################################//

    double transform_angle_dr = env.home[r_number].goal.angle;
    double inv_op_angle_dr=op_angle_dr+180;// let tail be head
    if(inv_op_angle_dr>180){
        inv_op_angle_dr=inv_op_angle_dr-360;
    }

    if(inv_op_angle_dr>100&&transform_angle_dr<-100){//+and -
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }else if(inv_op_angle_dr<-100&&transform_angle_dr>100){
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }

    //#######################################################//
    //                                                       //
    //    giving speed
    //    1.v_yaw = (head to angle_dr) + (tail to op_angle_dr)
    //    2.v_x = move to define_middle_line_dis base on where you are
    //    3.v_y = y_distance_br*1.5-y_distance_opdr
    //    ## all detect number need to below 3.5 meters, or will not correct
    //                                                       //
    //#######################################################//

    //---------------------------------------yaw_speed--------------------------
    //yaw_speed = inv_op_angle_dr+transform_angle_dr; // (head to angle_dr) + (tail to op_angle_dr) = face forward

    //---------------------------------------x_speed---------------------------
    if((op_angle_dr<0&&angle_dr<0)&&(angle_br>=0)){//robot && ball left side, so ready to "move to right" side define_dis
        x_moving =2;
    }else if((op_angle_dr>=0&&angle_dr>=0)&&(angle_br<0)){//robot && ball right side, so ready to "move to left" side define_dis
        x_moving =1;
    }

    if(fabs(inv_op_angle_dr+angle_dr)<20){// if face forward correct within error interval, give x vector speed
        switch(x_moving){
            case 1://move to left
                printf("move to left\n");
                x_speed = -to_middle_line_dis-define_middle_line_dis;
                if(fabs(x_speed)<0.1){
                    x_speed = 0;
                    if(x_distance_br<x_distance_opdr){//ball across field
                        x_moving=2;
                    }
                }
            break;
            case 2://move to right
                printf("move to right\n");
                x_speed = -to_middle_line_dis+define_middle_line_dis;
                if(fabs(x_speed)<0.1){
                    x_speed = 0;
                    if(x_distance_br<x_distance_opdr){//ball across field
                        x_moving=1;
                    }
                }
            break;
            default:
                if(op_angle_dr<0 && angle_dr<0){// left side, move to left side position
                    printf("move to left\n");
                    x_speed = -to_middle_line_dis-define_middle_line_dis;
                    if(fabs(x_speed)<0.1){
                        x_speed = 0;
                        if(x_distance_br<x_distance_opdr){//ball across field
                            x_moving=2;
                        }
                    }
                }else if(op_angle_dr>=0 && angle_dr>=0){// right side, move to right side position
                    printf("move to right\n");
                    x_speed = -to_middle_line_dis+define_middle_line_dis;
                    if(fabs(x_speed)<0.1){//almost to right position
                        x_speed = 0;
                        if(x_distance_br<x_distance_opdr){//ball across field
                            x_moving=1;
                        }
                    }
                }
            break;

        }
        //------------------------y_speed-------------------------

        if(y_distance_opdr<3&& op_distance_dr<3.4){// if not across defend gate
            if(cos(angle_br*deg2rad)>=0){//if ball in front of robot
              y_speed = y_distance_br*2-y_distance_opdr; // give y speed
              if(y_distance_br>1.5 && y_distance_opdr>=2.5){//can reach farest distance
                  printf("farest distance\n");
                  y_speed = 0;
              }else if(fabs(y_distance_br*2-y_distance_opdr)<0.2){
                  printf("moving speed too small\n");
                  y_speed = 0;
              }else{
                  printf("ball in front of robot\n");
              }
            }else{//ball at robot back
               printf("back\n");
              y_speed =-y_distance_opdr;
            }

        }else{// across defend gate, move back
            printf("too far from defend gate\n");
            if(op_distance_dr>=3.4){
              y_speed = -op_distance_dr+3;
            }else if(y_distance_opdr>=3){
              y_speed = -y_distance_opdr+3;
            }
        }


    }else{
        x_speed =0;
        y_speed =0;
        yaw_speed = inv_op_angle_dr+transform_angle_dr;
    }

    env.home[r_number].v_yaw = yaw_speed*2;
    env.home[r_number].v_x = -x_speed*speed_constant;
    env.home[r_number].v_y = y_speed*speed_constant;

    //#######################################################//
    //                                                       //
    //    lowest speed limit (if to low, set it to zero)
    //                                                       //
    //#######################################################//

    if(fabs(yaw_speed)<yaw_speed_limit){
       env.home[r_number].v_yaw= 0;
    }
    if((fabs(x_speed*speed_constant)<speed_limit)&&(fabs(y_speed*speed_constant)<speed_limit)){
       env.home[r_number].v_x = 0;
       env.home[r_number].v_y = 0;
    }
    shoot = 0;

//    printf("x_speed*speed_constant=%f\n",x_speed*speed_constant);
//    printf("y_speed*speed_constant=%f\n",y_speed*speed_constant);
//    printf("yaw_speed=%f\n",yaw_speed);

//    printf("FFFFf=%f\n",fabs(y_distance_br*2-y_distance_opdr));
//    printf("y_distance_br=%f\n",y_distance_br);
//    printf("y_distance_opdr=%f\n",y_distance_opdr);
//    printf("y_distance_dr=%f\n",y_distance_dr);
//    printf("x_distance_opdr=%f\n",x_distance_opdr);
//    printf("x_distance_br=%f\n",x_distance_br);


}
void FIRA_pathplan_class::strategy_Support_LostInternet(int r_number){

    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double x_speed = 0;
    double y_speed = 0;
    double yaw_speed = 0;
    double to_middle_line_dis = sin(op_angle_dr*deg2rad)*op_distance_dr;
    static int x_moving=0;
    double define_middle_line_dis =0.5;
    double y_distance_dr=fabs(cos(angle_dr*deg2rad))*distance_dr;
    double y_distance_opdr=fabs(cos(op_angle_dr*deg2rad))*op_distance_dr;
    double y_distance_br=fabs(cos(angle_br*deg2rad))*distance_br;

    //#######################################################//
    //                                                       //
    //    angle transform
    //    1.inv_op_angle_dr = tail face op_angle_dr
    //    2.transform_angle_dr = angle_dr
    //                                                       //
    //#######################################################//

            double transform_angle_dr = env.home[r_number].goal.angle;
            double inv_op_angle_dr=op_angle_dr+180;// let tail be head
            if(inv_op_angle_dr>180){
                inv_op_angle_dr=inv_op_angle_dr-360;
            }

            if(inv_op_angle_dr>100&&transform_angle_dr<-100){//+and -
                if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
                    inv_op_angle_dr=0;
                }else{
                    transform_angle_dr=0;
                }
            }else if(inv_op_angle_dr<-100&&transform_angle_dr>100){
                if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
                    inv_op_angle_dr=0;
                }else{
                    transform_angle_dr=0;
                }
            }

    if((op_angle_dr<0&&angle_dr<0)&&(angle_br>=0)){//robot && ball left side
//        printf("left side\n");
        x_moving =2;
    }else if((op_angle_dr>=0&&angle_dr>=0)&&(angle_br<0)){//robot && ball right side
//        printf("right side\n");
        x_moving =1;
    }

    //#######################################################//
    //                                                       //
    //    giving speed
    //    1.v_yaw = (head to angle_dr) + (tail to op_angle_dr)
    //    2.v_x =
    //    3.v_y =
    //    ##
    //                                                       //
    //#######################################################//

    //----------------------yaw_speed----------------------------------------------
    yaw_speed = inv_op_angle_dr+transform_angle_dr;

    static int switch_attacker=0;
    if(fabs(inv_op_angle_dr+angle_dr)<20){//if face forward
       if(switch_attacker==0){
           printf("switch_attacker==0\n");
           switch(x_moving){
               case 1://move to left
                   printf("move to left\n");
                   x_speed = -to_middle_line_dis-define_middle_line_dis;
               break;
               case 2://move to right
                   printf("move to right\n");
                   x_speed = -to_middle_line_dis+define_middle_line_dis;
               break;
               default:
                   if(op_angle_dr<0 && angle_dr<0){// left side, move to left side position
                       printf("move to left\n");
                       x_speed = -to_middle_line_dis-define_middle_line_dis;
                   }else if(op_angle_dr>=0 && angle_dr>=0){// right side, move to right side position
                       printf("move to right\n");
                       x_speed = -to_middle_line_dis+define_middle_line_dis;
                   }
               break;

           }
           if(y_distance_opdr<3.3){// if not across defend gate
               if(cos(angle_br*deg2rad)>=0){//ball in front of robot
                 y_speed = y_distance_br*2-y_distance_opdr;
                 if(y_distance_br>1.5 && y_distance_opdr>=3.2){
                     y_speed = 0;
                 }
                 printf("ball in front of robot\n");
               }else{//ball at robot back
                  printf("back\n");
                 y_speed = -y_distance_opdr;
               }

           }else{
               printf("too far from defend gate\n");
               y_speed = -y_distance_opdr;
           }

           if((cos(angle_br*deg2rad)>=0)&&(y_distance_opdr+y_distance_br<3.3)){
               //face forward and ball infront, if ball_dis+opdr<3
               //means ball must in our defense goal
//             angle_chase = Chase_Strategy[3];//16.5
//             distance_chase = Chase_Strategy[4];//0.4
               printf("change to attack state \n");
               switch_attacker=1;
           }
           shoot = 0;

       }else if(switch_attacker==1){
           printf("switch_attacker==1\n");
           if(op_distance_dr>3){
               switch_attacker=0;
               if(fabs(angle_br)<1 && distance_br<0.39){//catch ball state
                   //fabs(angle_br)<Chase_Strategy[3] && distance_br<Chase_Strategy[4]
                   shoot = SPlanning_Velocity[10];
               }else{
                   shoot = 0;
               }
           }else{
               if(fabs(angle_br)<1 && distance_br<0.39){//catch ball state
                   strategy_Attack(r_number);
                   printf("attackXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
               }else{
                   strategy_Chase(r_number);
                   printf("chaseXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxx\n");
               }
           }
       }

    }else{
        x_speed = 0;
        y_speed = 0;
        shoot = 0;
    }
    env.home[r_number].v_yaw = yaw_speed;
    env.home[r_number].v_x = x_speed*speed_constant;
    env.home[r_number].v_y = y_speed*speed_constant;

    //#######################################################//
    //                                                       //
    //    lowest speed limit (if to low, set it to zero)
    //                                                       //
    //#######################################################//

    if(fabs(yaw_speed)<yaw_speed_limit){
       env.home[r_number].v_yaw= 0;
    }
    if((fabs(x_speed*speed_constant)<speed_limit)&&(fabs(y_speed*speed_constant)<speed_limit)){
       env.home[r_number].v_x = 0;
       env.home[r_number].v_y = 0;
    }
    shoot = 0;

//    printf("x_speed*speed_constant=%f\n",x_speed*speed_constant);
//    printf("y_speed*speed_constant=%f\n",y_speed*speed_constant);
//    printf("yaw_speed=%f\n",yaw_speed);

//    printf("FFFFf=%f\n",inv_op_angle_dr+transform_angle_dr);
//    printf("y_distance_br=%f\n",y_distance_br);
//    printf("y_distance_opdr=%f\n",y_distance_opdr);
//    printf("y_distance_dr=%f\n",y_distance_dr);
//    printf("cos(angle_br*deg2rad)=%f\n",cos(angle_br*deg2rad));
//    printf("ball dis =%f\n",distance_br);
//    printf("angle br=%f\n",angle_br);


}

void FIRA_pathplan_class::strategy_MovetoGoal(int r_number){

    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double v_rotation= 0;// to give robot head direction x & y speed to op_angle_dr
    double x_speed = 0;
    double y_speed = 0;
    double yaw_speed = 0;

    //#######################################################//
    //                                                       //
    //    angle transform
    //    1.inv_op_angle_dr = tail face op_angle_dr
    //    2.transform_angle_dr = angle_dr
    //    3.v_rotation = to give robot head direction x & y speed to op_angle_dr
    //                                                       //
    //#######################################################//

    double transform_angle_dr = env.home[r_number].goal.angle;
    double inv_op_angle_dr=op_angle_dr+180;// let tail be head
    if(inv_op_angle_dr>180){
        inv_op_angle_dr=inv_op_angle_dr-360;
    }

    if(inv_op_angle_dr>100&&transform_angle_dr<-100){//+and -
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }else if(inv_op_angle_dr<-100&&transform_angle_dr>100){
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }

    //#######################################################//
    //                                                       //
    //    giving speed
    //    1.v_yaw = (head to angle_dr) + (tail to op_angle_dr)
    //    2.v_x =
    //    3.v_y =
    //                                                       //
    //#######################################################//

    //-----------------------yaw_speed-------------------
    yaw_speed =inv_op_angle_dr+transform_angle_dr;

    //-----------------------x_speed---------------------
    //-----------------------y_speed---------------------

    v_rotation=angle_dr+90;
    if(v_rotation>180){
        v_rotation=-360+v_rotation;
    }
    if(v_rotation<-180){
        v_rotation=360+v_rotation;
    }
    x_speed = cos(v_rotation*deg2rad)*distance_dr;
    y_speed = sin(v_rotation*deg2rad)*distance_dr;


    env.home[r_number].v_yaw = yaw_speed;
    env.home[r_number].v_x = x_speed*speed_constant;
    env.home[r_number].v_y = y_speed*speed_constant;

    //#######################################################//
    //                                                       //
    //    lowest speed limit (if to low, set it to zero)
    //                                                       //
    //#######################################################//

    if(fabs(yaw_speed)<yaw_speed_limit){
       env.home[r_number].v_yaw= 0;
    }
    if((fabs(x_speed*speed_constant)<speed_limit)&&(fabs(y_speed*speed_constant)<speed_limit)){
       env.home[r_number].v_x = 0;
       env.home[r_number].v_y = 0;
    }
    shoot = 0;

}
void FIRA_pathplan_class::strategy_MovetoOpGoal(int r_number){

    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double v_rotation= 0;// to give robot head direction x & y speed to op_angle_dr
    double x_speed = 0;
    double y_speed = 0;
    double yaw_speed = 0;

    //#######################################################//
    //                                                       //
    //    angle transform
    //    1.inv_op_angle_dr = tail face op_angle_dr
    //    2.transform_angle_dr = angle_dr
    //    3.v_rotation = to give robot head direction x & y speed to op_angle_dr
    //                                                       //
    //#######################################################//

    double transform_angle_dr = env.home[r_number].goal.angle;
    double inv_op_angle_dr=op_angle_dr+180;// let tail be head
    if(inv_op_angle_dr>180){
        inv_op_angle_dr=inv_op_angle_dr-360;
    }

    if(inv_op_angle_dr>100&&transform_angle_dr<-100){//+and -
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }else if(inv_op_angle_dr<-100&&transform_angle_dr>100){
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }

    //#######################################################//
    //                                                       //
    //    giving speed
    //    1.v_yaw = (head to angle_dr) + (tail to op_angle_dr)
    //    2.v_x =
    //    3.v_y =
    //                                                       //
    //#######################################################//

    //-----------------------yaw_speed-------------------
    yaw_speed =inv_op_angle_dr+transform_angle_dr;

    //-----------------------x_speed---------------------
    //-----------------------y_speed---------------------

    v_rotation=op_angle_dr+90;
    if(v_rotation>180){
        v_rotation=-360+v_rotation;
    }
    if(v_rotation<-180){
        v_rotation=360+v_rotation;
    }
    x_speed = cos(v_rotation*deg2rad)*distance_dr;
    y_speed = sin(v_rotation*deg2rad)*distance_dr;


    env.home[r_number].v_yaw = yaw_speed;
    env.home[r_number].v_x = x_speed*speed_constant;
    env.home[r_number].v_y = y_speed*speed_constant;

    //#######################################################//
    //                                                       //
    //    lowest speed limit (if to low, set it to zero)
    //                                                       //
    //#######################################################//

    if(fabs(yaw_speed)<yaw_speed_limit){
       env.home[r_number].v_yaw= 0;
    }
    if((fabs(x_speed*speed_constant)<speed_limit)&&(fabs(y_speed*speed_constant)<speed_limit)){
       env.home[r_number].v_x = 0;
       env.home[r_number].v_y = 0;
    }
    shoot = 0;
}

void FIRA_pathplan_class::strategy_MovetoGoalEdge1(int r_number){
    // move to goal left;
    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double v_rotation= 0;// to give robot head direction x & y speed to op_angle_dr
    double x_speed = 0;
    double y_speed = 0;
    double yaw_speed = 0;

    //#######################################################//
    //                                                       //
    //    angle transform
    //    1.inv_op_angle_dr = tail face op_angle_dr
    //    2.transform_angle_dr = angle_dr
    //    3.v_rotation = to give robot head direction x & y speed to op_angle_dr
    //                                                       //
    //#######################################################//

    double transform_angle_dr = env.home[r_number].goal.angle;
    double inv_op_angle_dr=op_angle_dr+180;// let tail be head
    if(inv_op_angle_dr>180){
        inv_op_angle_dr=inv_op_angle_dr-360;
    }

    if(inv_op_angle_dr>100&&transform_angle_dr<-100){//+and -
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }else if(inv_op_angle_dr<-100&&transform_angle_dr>100){
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }

    //#######################################################//
    //                                                       //
    //    giving speed
    //    1.v_yaw = (head to angle_dr) + (tail to op_angle_dr)
    //    2.v_x =
    //    3.v_y =
    //                                                       //
    //#######################################################//

    //-----------------------yaw_speed-------------------
    yaw_speed =inv_op_angle_dr+transform_angle_dr;

    //-----------------------x_speed---------------------
    //-----------------------y_speed---------------------

    v_rotation=env.home[r_number].goal_edge.angle_1+90;
    if(v_rotation>180){
        v_rotation=-360+v_rotation;
    }
    if(v_rotation<-180){
        v_rotation=360+v_rotation;
    }
    x_speed = cos(v_rotation*deg2rad)*distance_dr;
    y_speed = sin(v_rotation*deg2rad)*distance_dr;

    double x_control=0;
    if(fabs(inv_op_angle_dr+transform_angle_dr)<20){
        x_control=10;
    }else{
        x_control=0;
    }
    env.home[r_number].v_yaw = yaw_speed;
    env.home[r_number].v_x = x_speed*speed_constant-x_control;
    env.home[r_number].v_y = y_speed*speed_constant;

    //#######################################################//
    //                                                       //
    //    lowest speed limit (if to low, set it to zero)
    //                                                       //
    //#######################################################//

    if(fabs(yaw_speed)<yaw_speed_limit){
       env.home[r_number].v_yaw= 0;
    }
    if((fabs(x_speed*speed_constant)<speed_limit)&&(fabs(y_speed*speed_constant)<speed_limit)){
       env.home[r_number].v_x = 0;
       env.home[r_number].v_y = 0;
    }
    shoot = 0;

}
void FIRA_pathplan_class::strategy_MovetoGoalEdge2(int r_number){
    // move to goal right
    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double v_rotation= 0;// to give robot head direction x & y speed to op_angle_dr
    double x_speed = 0;
    double y_speed = 0;
    double yaw_speed = 0;

    //#######################################################//
    //                                                       //
    //    angle transform
    //    1.inv_op_angle_dr = tail face op_angle_dr
    //    2.transform_angle_dr = angle_dr
    //    3.v_rotation = to give robot head direction x & y speed to op_angle_dr
    //                                                       //
    //#######################################################//

    double transform_angle_dr = env.home[r_number].goal.angle;
    double inv_op_angle_dr=op_angle_dr+180;// let tail be head
    if(inv_op_angle_dr>180){
        inv_op_angle_dr=inv_op_angle_dr-360;
    }

    if(inv_op_angle_dr>100&&transform_angle_dr<-100){//+and -
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }else if(inv_op_angle_dr<-100&&transform_angle_dr>100){
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }

    //#######################################################//
    //                                                       //
    //    giving speed
    //    1.v_yaw = (head to angle_dr) + (tail to op_angle_dr)
    //    2.v_x =
    //    3.v_y =
    //                                                       //
    //#######################################################//

    //-----------------------yaw_speed-------------------
    yaw_speed =inv_op_angle_dr+transform_angle_dr;

    //-----------------------x_speed---------------------
    //-----------------------y_speed---------------------

    v_rotation=env.home[r_number].goal_edge.angle_2+90;
    if(v_rotation>180){
        v_rotation=-360+v_rotation;
    }
    if(v_rotation<-180){
        v_rotation=360+v_rotation;
    }
    x_speed = cos(v_rotation*deg2rad)*distance_dr;
    y_speed = sin(v_rotation*deg2rad)*distance_dr;

    double x_control=0;
    if(fabs(inv_op_angle_dr+transform_angle_dr)<20){
        x_control=10;
    }else{
        x_control=0;
    }
    env.home[r_number].v_yaw = yaw_speed;
    env.home[r_number].v_x = x_speed*speed_constant+x_control;
    env.home[r_number].v_y = y_speed*speed_constant;

    //#######################################################//
    //                                                       //
    //    lowest speed limit (if to low, set it to zero)
    //                                                       //
    //#######################################################//

    if(fabs(yaw_speed)<yaw_speed_limit){
       env.home[r_number].v_yaw= 0;
    }
    if((fabs(x_speed*speed_constant)<speed_limit)&&(fabs(y_speed*speed_constant)<speed_limit)){
       env.home[r_number].v_x = 0;
       env.home[r_number].v_y = 0;
    }
    shoot = 0;

}

void FIRA_pathplan_class::strategy_MovetoOpGoalEdge1(int r_number){

    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double v_rotation= 0;// to give robot head direction x & y speed to op_angle_dr
    double x_speed = 0;
    double y_speed = 0;
    double yaw_speed = 0;

    //#######################################################//
    //                                                       //
    //    angle transform
    //    1.inv_op_angle_dr = tail face op_angle_dr
    //    2.transform_angle_dr = angle_dr
    //    3.v_rotation = to give robot head direction x & y speed to op_angle_dr
    //                                                       //
    //#######################################################//

    double transform_angle_dr = env.home[r_number].goal.angle;
    double inv_op_angle_dr=op_angle_dr+180;// let tail be head
    if(inv_op_angle_dr>180){
        inv_op_angle_dr=inv_op_angle_dr-360;
    }

    if(inv_op_angle_dr>100&&transform_angle_dr<-100){//+and -
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }else if(inv_op_angle_dr<-100&&transform_angle_dr>100){
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }

    //#######################################################//
    //                                                       //
    //    giving speed
    //    1.v_yaw = (head to angle_dr) + (tail to op_angle_dr)
    //    2.v_x =
    //    3.v_y =
    //                                                       //
    //#######################################################//

    //-----------------------yaw_speed-------------------
    yaw_speed =inv_op_angle_dr+transform_angle_dr;

    //-----------------------x_speed---------------------
    //-----------------------y_speed---------------------

    v_rotation=env.home[r_number].op_goal_edge.angle_1+90;
    if(v_rotation>180){
        v_rotation=-360+v_rotation;
    }
    if(v_rotation<-180){
        v_rotation=360+v_rotation;
    }
    x_speed = cos(v_rotation*deg2rad)*distance_dr;
    y_speed = sin(v_rotation*deg2rad)*distance_dr;


    env.home[r_number].v_yaw = yaw_speed;
    env.home[r_number].v_x = x_speed*speed_constant;
    env.home[r_number].v_y = y_speed*speed_constant;

    //#######################################################//
    //                                                       //
    //    lowest speed limit (if to low, set it to zero)
    //                                                       //
    //#######################################################//

    if(fabs(yaw_speed)<yaw_speed_limit){
       env.home[r_number].v_yaw= 0;
    }
    if((fabs(x_speed*speed_constant)<speed_limit)&&(fabs(y_speed*speed_constant)<speed_limit)){
       env.home[r_number].v_x = 0;
       env.home[r_number].v_y = 0;
    }
    shoot = 0;

}

void FIRA_pathplan_class::strategy_MovetoOpGoalEdge2(int r_number){

    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    double v_rotation= 0;// to give robot head direction x & y speed to op_angle_dr
    double x_speed = 0;
    double y_speed = 0;
    double yaw_speed = 0;

    //#######################################################//
    //                                                       //
    //    angle transform
    //    1.inv_op_angle_dr = tail face op_angle_dr
    //    2.transform_angle_dr = angle_dr
    //    3.v_rotation = to give robot head direction x & y speed to op_angle_dr
    //                                                       //
    //#######################################################//

    double transform_angle_dr = env.home[r_number].goal.angle;
    double inv_op_angle_dr=op_angle_dr+180;// let tail be head
    if(inv_op_angle_dr>180){
        inv_op_angle_dr=inv_op_angle_dr-360;
    }

    if(inv_op_angle_dr>100&&transform_angle_dr<-100){//+and -
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }else if(inv_op_angle_dr<-100&&transform_angle_dr>100){
        if(fabs(inv_op_angle_dr)>fabs(transform_angle_dr)){//turn to smaller way
            inv_op_angle_dr=0;
        }else{
            transform_angle_dr=0;
        }
    }

    //#######################################################//
    //                                                       //
    //    giving speed
    //    1.v_yaw = (head to angle_dr) + (tail to op_angle_dr)
    //    2.v_x =
    //    3.v_y =
    //                                                       //
    //#######################################################//

    //-----------------------yaw_speed-------------------
    yaw_speed =inv_op_angle_dr+transform_angle_dr;

    //-----------------------x_speed---------------------
    //-----------------------y_speed---------------------

    v_rotation=env.home[r_number].op_goal_edge.angle_2+90;
    if(v_rotation>180){
        v_rotation=-360+v_rotation;
    }
    if(v_rotation<-180){
        v_rotation=360+v_rotation;
    }
    x_speed = cos(v_rotation*deg2rad)*distance_dr;
    y_speed = sin(v_rotation*deg2rad)*distance_dr;


    env.home[r_number].v_yaw = yaw_speed;
    env.home[r_number].v_x = x_speed*speed_constant;
    env.home[r_number].v_y = y_speed*speed_constant;

    //#######################################################//
    //                                                       //
    //    lowest speed limit (if to low, set it to zero)
    //                                                       //
    //#######################################################//

    if(fabs(yaw_speed)<yaw_speed_limit){
       env.home[r_number].v_yaw= 0;
    }
    if((fabs(x_speed*speed_constant)<speed_limit)&&(fabs(y_speed*speed_constant)<speed_limit)){
       env.home[r_number].v_x = 0;
       env.home[r_number].v_y = 0;
    }
    shoot = 0;

}

void FIRA_pathplan_class::strategy_Stop(int r_number){
    // move to goal left;
    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;
    env.home[r_number].v_yaw= 0;
    env.home[r_number].v_x = 0;
    env.home[r_number].v_y = 0;
     shoot = 0;
}

void FIRA_pathplan_class::strategy_Block(int r_number){
    double distance_br = env.home[r_number].ball.distance;
    double distance_dr = env.home[r_number].goal.distance;
    double op_distance_dr = env.home[r_number].op_goal.distance;
    double angle_br = env.home[r_number].ball.angle;
    double angle_dr = env.home[r_number].goal.angle;
    double op_angle_dr = env.home[r_number].op_goal.angle;

    double obstacle_angle =env.Support_Obstacle_angle+90;
    double transform_angle_br = angle_br+90;
    double obstacle_distance =env.Support_Obstacle_distance;
    double transform_obstacle_angle = env.Support_Obstacle_angle + 180;
    Current_time = ros::Time::now().toSec();
    if(transform_obstacle_angle>180){
        transform_obstacle_angle = transform_obstacle_angle -360;
    }else if(transform_obstacle_angle<-180){
        transform_obstacle_angle = transform_obstacle_angle +360;
    }
    printf("transform_obstacle_angle=%f\n",transform_obstacle_angle);

    if(obstacle_angle>180){
        obstacle_angle=obstacle_angle-360;
    }else if(obstacle_angle<-180){
        obstacle_angle=obstacle_angle+360;
    }

    if(transform_angle_br>180){
        transform_angle_br=transform_angle_br-360;
    }else if(transform_angle_br<-180){
        transform_angle_br=transform_angle_br+360;
    }
    double yaw_speed=(fabs(transform_obstacle_angle)<15)? 0 : transform_obstacle_angle;
    if(env.Support_Obstacle_angle>=999||env.Support_Obstacle_distance>=9.99){
        env.home[r_number].v_x = 0;
        env.home[r_number].v_y = 0;
        env.home[r_number].v_yaw= 0;
    }else{
        if(fabs(Current_time-Begin_time)<2){
            env.home[r_number].v_x =cos(obstacle_angle*deg2rad)*obstacle_distance+cos(transform_angle_br*deg2rad)*distance_br;
            env.home[r_number].v_y =sin(obstacle_angle*deg2rad)*obstacle_distance+sin(transform_angle_br*deg2rad)*distance_br;
            env.home[r_number].v_yaw= yaw_speed;
        }else{
            env.home[r_number].v_x =cos(obstacle_angle*deg2rad)*(obstacle_distance-0.6);
            env.home[r_number].v_y =sin(obstacle_angle*deg2rad)*(obstacle_distance-0.6);
            env.home[r_number].v_yaw= yaw_speed;
            if(obstacle_distance<0.6){
                env.home[r_number].v_x =0;
                env.home[r_number].v_y =0;
                printf("obstacle_distance<0.6\n");
            }
        }
    }
//    printf("fabs(Current_time-Begin_time)=%f\n",fabs(Current_time-Begin_time));
//    printf("obstacle_angle=%f\n",env.Support_Obstacle_angle);
//    printf("obstacle_distance=%f\n",env.Support_Obstacle_distance);

    if(fabs(yaw_speed)<yaw_speed_limit){
       env.home[r_number].v_yaw= 0;
    }
//    if(fabs(cos(obstacle_angle*deg2rad)*(obstacle_distance-0.5)<speed_limit)&&fabs(sin(obstacle_angle*deg2rad)*(obstacle_distance-0.5)<speed_limit)){
//       env.home[r_number].v_x = 0;
//       env.home[r_number].v_y = 0;
//    }
     shoot = 0;



}
void FIRA_pathplan_class::strategy_Kick(int Robot_index){
    if(SPlanning_Velocity[11]<=100){
        shoot = SPlanning_Velocity[11];
    }else{
        shoot = 100;
    }
    printf("shoot=%d\n",shoot);
}

void FIRA_pathplan_class::strategy_FreeKick(int Robot_index){
    printf("FreeKick\n");
    env.home[Robot_index].v_x = 0;
    if(env.Support_WhiteLine_distance>0.4){
        env.home[Robot_index].v_y = -env.Support_WhiteLine_distance;
    }else{
        env.home[Robot_index].v_y = 0;
    }
    env.home[Robot_index].v_yaw= 0;

}

//==========for ROS special===============//
//###################################################//
//                                                   //
//                 load parameter                    //
//                                                   //
//###################################################//
void FIRA_pathplan_class::loadParam(ros::NodeHandle *n){
    if(n->getParam("/FIRA/Attack_Strategy", Attack_Strategy)){
//        for(int i=0;i<1;i++)
//            std::cout<< "param Attack_Strategy["<< i << "]=" << Attack_Strategy[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA_Behavior/Chase_Strategy", Chase_Strategy)){
//        for(int i=0;i<1;i++)
//            std::cout<< "param Chase_Strategy["<< i << "]=" << Chase_Strategy[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA/Zone_Attack", Zone_Attack)){
//        for(int i=0;i<2;i++)
//            std::cout<< "param Zone_Attack["<< i << "]=" << Zone_Attack[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
   if(n->getParam("/FIRA/TypeS_Attack", TypeS_Attack)){
//       for(int i=0;i<2;i++)
//           std::cout<< "param TypeS_Attack["<< i << "]=" << TypeS_Attack[i] << std::endl;
//   std::cout << "====================================" << std::endl;
   }
   if(n->getParam("/FIRA/TypeU_Attack", TypeU_Attack)){
//       for(int i=0;i<8;i++)
//           std::cout<< "param TypeU_Attack["<< i << "]=" << TypeU_Attack[i] << std::endl;
//   std::cout << "====================================" << std::endl;
   }
   if(n->getParam("/FIRA/Dorsad_Attack", Dorsad_Attack)){
   }
   if(n->getParam("/FIRA/Corner_Kick", Corner_Kick)){
   }
   if(n->getParam("/FIRA/Penalty_Kick", Penalty_Kick)){
   }
   if(n->getParam("/FIRA/SideSpeedUp", Side_Speed_Up)){
//       for(int i=0;i<5;i++)
//           std::cout<< "param SideSpeedUp["<< i << "]=" << SideSpeedUp[i] << std::endl;
//   std::cout << "====================================" << std::endl;
   }
   if(n->getParam("/FIRA/SPlanning_Velocity", SPlanning_Velocity)){
  //     for(int i=0;i<8;i++)
  //         std::cout<< "param SPlanning_Velocity["<< i << "]=" << SPlanning_Velocity[i] << std::endl;
  // std::cout << "====================================" << std::endl;
   }
//    std::string ns = "/FIRA/R1/Strategy/Pathplan/";

//    if(n->getParam("/FIRA/TeamColor",teamColor)){
//        std::cout << "param teamColor=" << teamColor <<std::endl;
//    }
//    if(n->getParam(ns + "beta_const",beta_const)){
//        std::cout << "beta_const=" << beta_const <<std::endl;
//    }
//    if(n->getParam(ns + "long_rush_alpha",long_rush_alpha)){
//        std::cout << "long_rush_alpha=" << long_rush_alpha <<std::endl;
//    }
//    if(n->getParam(ns + "long_rush_speed_const",long_rush_speed_const)){
//        std::cout << "long_rush_speed_const=" << long_rush_speed_const <<std::endl;
//    }
//    if(n->getParam(ns + "long_rush_dis_br",long_rush_dis_br)){
//        std::cout << "long_rush_dis_br=" << long_rush_dis_br <<std::endl;
//    }
//    if(n->getParam(ns + "short_rush_dis_dr",short_rush_dis_dr)){
//        std::cout << "short_rush_dis_dr=" << short_rush_dis_dr <<std::endl;
//    }
//    if(n->getParam(ns + "short_rush_alpha",short_rush_alpha)){
//        std::cout << "short_rush_alpha=" << short_rush_alpha <<std::endl;
//    }
//    if(n->getParam(ns + "short_rush_dis_br",short_rush_dis_br)){
//        std::cout << "short_rush_dis_br=" << short_rush_dis_br <<std::endl;
//    }
//    if(n->getParam(ns + "short_rush_speed_const",short_rush_speed_const)){
//        std::cout << "short_rush_speed_const=" << short_rush_speed_const <<std::endl;
//    }
//    if(n->getParam(ns + "close_ball_speed_const",close_ball_speed_const)){
//        std::cout << "close_ball_speed_const=" << close_ball_speed_const <<std::endl;
//    }
//    if(n->getParam(ns + "close_ball_dis_const",close_ball_dis_const)){
//        std::cout << "close_ball_dis_const=" << close_ball_dis_const <<std::endl;
//    }
//    if(n->getParam(ns + "far_ball_speed_const",far_ball_speed_const)){
//        std::cout << "far_ball_speed_const=" << far_ball_speed_const <<std::endl;
//    }
//    if(n->getParam(ns + "head2ball_speed",head2ball_speed)){
//        std::cout << "head2ball_speed=" << head2ball_speed <<std::endl;
//    }
//    //------------------------GOALKEEPER_CONST------------------------------------
//    if(n->getParam(ns + "goalkeeper_radius",goalkeeper_radius)){
//        std::cout << "goalkeeper_radius=" << goalkeeper_radius <<std::endl;
//    }
//    if(n->getParam(ns + "goalkeeper_front_dis",goalkeeper_front_dis)){
//        std::cout << "goalkeeper_front_dis=" << goalkeeper_front_dis <<std::endl;
//    }
//    if(n->getParam(ns + "goalkeeper_mid_dis",goalkeeper_mid_dis)){
//        std::cout << "goalkeeper_mid_dis=" << goalkeeper_mid_dis <<std::endl;
//    }
//    if(n->getParam(ns + "goalkeeper_side_dis",goalkeeper_side_dis)){
//        std::cout << "goalkeeper_side_dis=" << goalkeeper_side_dis <<std::endl;
//    }
//    if(n->getParam(ns + "goalkeeper_front_angle",goalkeeper_front_angle)){
//        std::cout << "goalkeeper_front_angle=" << goalkeeper_front_angle <<std::endl;
//    }
//    if(n->getParam(ns + "goalkeeper_mid_angle",goalkeeper_mid_angle)){
//        std::cout << "goalkeeper_mid_angle=" << goalkeeper_mid_angle <<std::endl;
//    }
//    if(n->getParam(ns + "goalkeeper_front_speed",goalkeeper_front_speed)){
//        std::cout << "goalkeeper_front_speed=" << goalkeeper_front_speed <<std::endl;
//    }
//    if(n->getParam(ns + "goalkeeper_mid_speed",goalkeeper_mid_speed)){
//        std::cout << "goalkeeper_mid_speed=" << goalkeeper_mid_speed <<std::endl;
//    }
//    if(n->getParam(ns + "goalkeeper_side_speed",goalkeeper_side_speed)){
//        std::cout << "goalkeeper_side_speed=" << goalkeeper_side_speed <<std::endl;
//    }

}
