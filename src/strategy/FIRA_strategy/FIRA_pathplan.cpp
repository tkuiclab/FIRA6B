#include "FIRA_pathplan.h"
#include "math.h"
#include "time.h"

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

            case action_Goalkeeper_init:
                strategy_Goalkeeper_init(robotIndex);
                break;
            case action_Goalkeeper_block:
                strategy_Goalkeeper_block(robotIndex);
                break;
            case action_Goalkeeper_push:
                strategy_Goalkeeper_push(robotIndex);
                break;
            case action_Goalkeeper_goalkick:
                strategy_Goalkeeper_goalkick(robotIndex);
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
//            case Role_PenaltyKick:;
//                strategy_PenaltyKick(robotIndex);
//                break;
//            case Role_ThrowIn:
//                strategy_ThrowIn(robotIndex);
//                break;
            case action_CornerKick:
                strategy_CornerKick(robotIndex);
                break;
            case  action_Zone_Attack:
                strategy_Zone_Attack(robotIndex);
                break;
        }
}
//###################################################//
//                                                   //
//                New path planning                  //
//                                                   //
//###################################################//
void FIRA_pathplan_class::strategy_Goalkeeper_init(int r_number){
    //do nothing
    env.home[r_number].v_x =0;
    env.home[r_number].v_y =0;
    env.home[r_number].v_yaw =0;
}

void FIRA_pathplan_class::strategy_Goalkeeper_block(int r_number){
    int position;
    int direction;
    std::string angle_fix;
    double ball_dis = env.home[r_number].ball.distance;
    double ball_angle = env.home[r_number].ball.angle;

    int opgoal_edge_angle1 = env.home[r_number].opgoal_edge.angle_1;
    int opgoal_edge_angle2 = env.home[r_number].opgoal_edge.angle_2;
    double opgoal_left = env.home[r_number].opgoal_edge.left_dis;
    double opgoal_right = env.home[r_number].opgoal_edge.right_dis;

    double opgoal_dis = env.home[r_number].op_goal.distance;
    double opgoal_angle = env.home[r_number].op_goal.angle;
    double ball_to_opgoal_dis;
    ball_to_opgoal_dis = (opgoal_angle*ball_angle)>0 ? fabs(opgoal_angle-ball_angle) : fabs(opgoal_angle)+fabs(ball_angle);
    if(ball_to_opgoal_dis == 180){
        ball_to_opgoal_dis = opgoal_dis + ball_dis;
    }else{
        if(ball_to_opgoal_dis > 180){
        ball_to_opgoal_dis = 360 - ball_to_opgoal_dis;
        }
        ball_to_opgoal_dis =sqrt((opgoal_dis*opgoal_dis)+(ball_dis*ball_dis)-(2*opgoal_dis*ball_dis*cos(ball_to_opgoal_dis*deg2rad)));
    }

    double x;
    double y;
    int rotAngle = 18;
    double speed = 0.21;
    double max_speed = 1.63;	//min = 0.21
    double opgoal_angle_reverse;
    if(opgoal_angle >177 || opgoal_angle < -173){
        opgoal_angle_reverse = 0;
     }else if(opgoal_angle > 0){
        opgoal_angle_reverse = opgoal_angle - 180;
    }else{
        opgoal_angle_reverse = 180 + opgoal_angle;
    }

    if(opgoal_dis > 1.2){
        rotAngle = -12;
    }
    //std::cout << "opgoal_dis = " << opgoal_dis << std::endl;
    //std::cout << "ball_to_opgoal_dis = " << ball_to_opgoal_dis << std::endl;
    if(fabs(opgoal_right-opgoal_left)< 0.35){ //on middle
        position = M;
        if(opgoal_dis > 1.25 && ball_to_opgoal_dis > 2.2){
            direction = max_M;
        }else if(ball_dis > 3.5){
            if(opgoal_dis > 0.75){
                direction = max_M;
            }else{
                direction = stop;
            }
        }else if(ball_angle > opgoal_angle_reverse + 15){
            rotAngle = -rotAngle;
            direction = L;
        }else if(ball_angle < opgoal_angle_reverse - 15){
            direction = R;
        }else{
            direction = stop;
        }
    }else if(opgoal_right > opgoal_left){ //on left
        position = L;
        if(opgoal_dis > 1.5 && ball_to_opgoal_dis > 2.2){
            if((opgoal_right - opgoal_left) > 0.45) {
                direction = max_R;
            }else{
                direction = max_M;
            }
        }else if(ball_dis > 3.5 && opgoal_dis > 0.75){
            if((opgoal_right - opgoal_left) > 0.45){
                direction = max_R;
            }else{
                direction = max_M;
            }
        }else if(ball_angle > opgoal_angle_reverse + 30){
            rotAngle = -rotAngle;
            direction = L;
        }else if(ball_angle < opgoal_angle_reverse + 10){
            direction = R;
        }else{
            direction = stop;
        }
    }else{  //on right
        position = R;
        if(opgoal_dis > 1.5 && ball_to_opgoal_dis > 2.2){
            if((opgoal_left - opgoal_right) > 0.45){
                direction = max_L;
            }else{
                direction = max_M;
            }
        }else if(ball_dis > 3.5 && opgoal_dis > 0.75){
            if((opgoal_left - opgoal_right) > 0.45){
                direction = max_L;
            }else{
                direction = max_M;
            }
        }else if(ball_angle > opgoal_angle_reverse - 10){
            rotAngle = -rotAngle;
            direction = L;
        }else if(ball_angle < opgoal_angle_reverse - 30){
            direction = R;
        }else{
            direction = stop;
        }
    }
    if(ball_to_opgoal_dis < 1.6){
        speed = 1.2;
    }else if(ball_to_opgoal_dis < 2){
        speed = 1.5;
    }
    //std::cout << "speed = " << speed << std::endl;

    switch(direction){
    case L:
        x = -speed * sin(opgoal_edge_angle2*deg2rad);
        y = speed * cos(opgoal_edge_angle2*deg2rad);
        break;
    case R:
        x = - speed * sin(opgoal_edge_angle1*deg2rad);
        y = speed * cos(opgoal_edge_angle1*deg2rad);
        break;
    case max_L:
        x = -max_speed * sin(opgoal_edge_angle2*deg2rad);
        y = max_speed * cos(opgoal_edge_angle2*deg2rad);
        break;
    case max_R:
        x = - max_speed * sin(opgoal_edge_angle1*deg2rad);
        y = max_speed * cos(opgoal_edge_angle1*deg2rad);
        break;
    case max_M:
        x = -max_speed *sin(opgoal_angle*deg2rad);
        y = max_speed *cos(opgoal_angle*deg2rad);
        break;
    case stop:
        x = 0;
        y = 0;
        break;
    }
    /****before Two_point left,right dis****/
    /*
    if(ball_angle > opgoal_angle_reverse){  //go left
        x = -speed * sin(opgoal_edge_angle2*deg2rad);
        y = speed * cos(opgoal_edge_angle2*deg2rad);
        rotAngle = -rotAngle;
        direction = "left";
        if(ball_angle < opgoal_angle_reverse+10){
            x =0;
            y =0;
            direction = "stop";
        }
    }else{    //go right
        x = - speed * sin(opgoal_edge_angle1*deg2rad);
        y = speed * cos(opgoal_edge_angle1*deg2rad);
        direction = "right";
        if(ball_angle > opgoal_angle_reverse-10){
            x =0;
            y =0;
            direction = "stop";
        }
    }

    if(opgoal_dis > 1.2){ //out of zoom pull back
        if(opgoal_dis <1.5){  //not good
            x = 2*(x - speed *sin(opgoal_angle*deg2rad));
            y = 2*(y + speed *cos(opgoal_angle*deg2rad));
            direction = "pullback";
        }else{
            x = -max_speed *sin(opgoal_angle*deg2rad);
            y = max_speed *cos(opgoal_angle*deg2rad);
            direction = "full speed pullback(out of zoom)";
        }
    }

    if(ball_dis > 4 && opgoal_dis > 0.7){
        x = -max_speed *sin(opgoal_angle*deg2rad);
        y = max_speed *cos(opgoal_angle*deg2rad);
        direction = "full speed pullback(can't see ball)";
    }

    */
    Vector2d vectorbr(x, y);
    Rotation2Dd rot( rotAngle * deg2rad);
    Vector2d vectornt = rot * vectorbr;

    //doesn't work well
//    if( fabs(opgoal_angle_reverse) > 40 ){
//        yaw = (opgoal_angle_reverse>0?1:-1)*10;
//        angle_fix = "angle_fix_middle";
//    }else if(opgoal_edge_angle2 < 30 && yaw > 0){
//        yaw = -20;
//        angle_fix = "angle_fix_left";
//    }else if(opgoal_edge_angle1 > -30 && yaw < 0){
//        yaw = -20;
//        angle_fix = "angle_fix_right";
//    }

    env.home[r_number].v_x =vectornt(0);
    env.home[r_number].v_y =vectornt(1);
    if(fabs(ball_angle) > 10){
        env.home[r_number].v_yaw = ball_angle *2;
    }
    std::cout << "block " << position << " -> " << direction << " " << angle_fix << std::endl;
}


void FIRA_pathplan_class::strategy_Goalkeeper_push(int r_number){

    int direction;
    double ball_dis = env.home[r_number].ball.distance;
    double ball_angle = env.home[r_number].ball.angle;
    double opgoal_dis = env.home[r_number].op_goal.distance;
    double opgoal_angle = env.home[r_number].op_goal.angle;
    int opgoal_edge_angle1 = env.home[r_number].opgoal_edge.angle_1;
    int opgoal_edge_angle2 = env.home[r_number].opgoal_edge.angle_2;
    double opgoal_angle_reverse;
    if(opgoal_angle > 0){
        opgoal_angle_reverse = opgoal_angle - 180;
    }else{
        opgoal_angle_reverse = 180 + opgoal_angle;
    }

    int max_speed = 3;
    double x = -max_speed * sin(ball_angle * deg2rad);
    double y = max_speed * cos(ball_angle * deg2rad);

    Vector2d vectorbr(x, y);
    double rotAngle = 0.001;
    if(ball_angle > opgoal_angle_reverse +25){
        rotAngle = opgoal_edge_angle2/2;
        direction = L;
    }else if(ball_angle < opgoal_angle_reverse -25){
        rotAngle = opgoal_edge_angle1/2;
        direction = R;
    }else{
        direction = M;
    }

//    std::cout << "angle_L = " << opgoal_edge_angle2 << std::endl;
//    std::cout << "angle_R = " << opgoal_edge_angle1 << std::endl;

//    if(opgoal_edge_angle2 < 30 && yaw > 0){
//        yaw = -10;
//        direction = direction + "+ angle_fix_left";
//    }else if(opgoal_edge_angle1 > -30 && yaw < 0){
//        yaw = 10;
//        direction = direction + "+ angle_fix_right";
//    }

    Rotation2Dd rot( rotAngle * deg2rad);
    Vector2d vectornt = rot * vectorbr;

    env.home[r_number].v_x = vectornt(0);
    env.home[r_number].v_y = vectornt(1);
    env.home[r_number].v_yaw = ball_angle*2;
    std::cout << "push -> " << direction << std::endl;
}

void FIRA_pathplan_class::strategy_Goalkeeper_goalkick(int r_number){
    int direction;
    double ball_dis = env.home[r_number].ball.distance;
    double ball_angle = env.home[r_number].ball.angle;
    double opgoal_dis = env.home[r_number].op_goal.distance;
    double opgoal_angle = env.home[r_number].op_goal.angle;
    double opgoal_left = env.home[r_number].opgoal_edge.left_dis;
    double opgoal_right = env.home[r_number].opgoal_edge.right_dis;
    int opgoal_edge_angle1 = env.home[r_number].opgoal_edge.angle_1;
    int opgoal_edge_angle2 = env.home[r_number].opgoal_edge.angle_2;
    double x ;
    double y ;  
    int rotAngle =0;
    std::cout << "opgoal = " << opgoal_dis << std::endl;
    if(opgoal_dis < 0.8){
	    x = 0.5 * sin(ball_angle * deg2rad);
	    y = 0.5 * cos(ball_angle * deg2rad);
        direction = M;      
    }else if(fabs(opgoal_left - opgoal_right) < 0.45){
    	x = 0;
	    y = 0;
        direction = stop;
    }else if(opgoal_left > opgoal_right){//L
        x = -0.3 * sin(opgoal_edge_angle2*deg2rad);
        y = 0.3 * cos(opgoal_edge_angle2*deg2rad);
	    rotAngle = -18;
        direction = L;
    }else{//R
        x = - 0.3 * sin(opgoal_edge_angle1*deg2rad);
        y = 0.3 * cos(opgoal_edge_angle1*deg2rad);
	    rotAngle = 18;
        direction = R;
    }
    Vector2d vectorbr(x, y);
    Rotation2Dd rot( rotAngle * deg2rad);
    Vector2d vectornt = rot * vectorbr;
    env.home[r_number].v_x = vectornt(0);
    env.home[r_number].v_y = vectornt(1);
    env.home[r_number].v_yaw = ball_angle;
    std::cout << "goalkick -> " << direction << std::endl;
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
    Rotation2Dd rot( rotAngle * deg2rad);
    Vector2d vectornt = rot * vectorbr;
    env.home[r_number].v_x =vectornt(0);
    env.home[r_number].v_y =vectornt(1);
    env.home[r_number].v_yaw = goal_angle;
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
    Vector2d vectordr(goal_x,goal_y);

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
    if(/*bool_shoot(goal_dis)&&fabs(ball_angle)<4&&*/fabs(goal_angle)<10){
        if(goal_dis<=1){
            shoot = 30;
}
        else if(goal_dis>1 && goal_dis <=2){
            shoot = 65;
}
        else {
            shoot = 185;
}
        env.home[r_number].v_x =vectornt(0)*1000;
        env.home[r_number].v_y =vectornt(1)*1000;
        env.home[r_number].v_yaw = goal_angle*2;
    }else{
        shoot = 0;
        env.home[r_number].v_x =vectornt(0)*1000;
        env.home[r_number].v_y =vectornt(1)*1000;
        env.home[r_number].v_yaw = goal_angle*2;
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

    if(/*bool_shoot(goal_dis)&&*//*fabs(ball_angle)<4&&*/fabs(goal_angle)<10){
        shoot = 185;
        env.home[r_number].v_x =0;
        env.home[r_number].v_y =0;
        env.home[r_number].v_yaw = goal_angle*2;
    }else{
        shoot = 0;
        env.home[r_number].v_x =0;
        env.home[r_number].v_y =0;
        env.home[r_number].v_yaw = goal_angle*2;
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
    shoot = 185;
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
        shoot = 185;
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
    double op_distance_rr = double(env.mindis[0]) / 100;
    int op_angle_rr = env.blackangle[0];

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
    }else if(op_distance_rr < distance2turn2){
       angle_Speed = angle_dorsad;// * 2/3;
       env.home[r_number].v_x = vectordr_x * (op_distance_rr / deceleration);
       env.home[r_number].v_y = vectordr_y * (op_distance_rr / deceleration);
    }else if(op_distance_rr < distance2turn1){
       angle_Speed = angle_vertical;/* * 2/3*/
       env.home[r_number].v_x = vectordr_x * (op_distance_rr / deceleration);
       env.home[r_number].v_y = vectordr_y * (op_distance_rr / deceleration);
    }else{
       angle_Speed = angle_dr; //* 2/3;
       env.home[r_number].v_x = vectordr_x;
       env.home[r_number].v_y = vectordr_y;
    }
    env.home[r_number].v_yaw = angle_Speed;

    if(fabs(angle_dr) < 7){ // if there is any chance, shoot!
        shoot = 185;
    }
}

void FIRA_pathplan_class::strategy_SideSpeedUp(int Robot_index){
    //-----------parameter--------------------------------------
    double beta_param = SideSpeedUp[0]; //0.73
    double const const_angle_mult = SideSpeedUp[1];//1.2
    double const const_dis = SideSpeedUp[2];//0.7
    double const const_mult = SideSpeedUp[3];//5.0
    double const const_only = SideSpeedUp[4];//2.2*0.8*10=17.6
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

}

void FIRA_pathplan_class::strategy_Halt(int Robot_index){
    env.home[Robot_index].v_x = 0;
    env.home[Robot_index].v_y = 0;
    env.home[Robot_index].v_yaw = 0;
    loopEnd = true;
}

void FIRA_pathplan_class::strategy_PenaltyKick(int Robot_index){

    Vector3D robot = env.home[Robot_index].pos;
    Vector3D ball = env.currentBall.pos;
    Vector3D goal = env.yellow.pos;

    double Robot_head_x = robot.x + half_robot*cos(env.home[Robot_index].rotation*deg2rad);
    double Robot_head_y = robot.y + half_robot*sin(env.home[Robot_index].rotation*deg2rad);

    double vectorbr_x = ball.x - Robot_head_x;
    double vectorbr_y = ball.y - Robot_head_y;

    double vectordr_x = goal.x - Robot_head_x;
    double vectordr_y = goal.y - Robot_head_y;

    double dis_br = hypot(vectorbr_x,vectorbr_y);
    double dis_dr = hypot(vectordr_x,vectordr_y);

    double v_x = vectordr_x*6;
    double v_y = vectordr_y*6;

    double phi = atan2(vectordr_y , vectordr_x) * rad2deg;
    double angle = phi - env.home[Robot_index].rotation;
    if(angle>180)
    {
        angle = angle - 360;
    }else if(angle < -180){
        angle = angle + 360;
    }

    if((fabs(robot.y) <= 1.5 && dis_br<=0.5)){
        env.home[1].v_x=v_x;
        env.home[1].v_y=v_y;
        env.home[1].v_yaw=angle;
    }else{
        strategy_Support(Robot_index);
    }
}

void FIRA_pathplan_class::strategy_ThrowIn(int Robot_index){
    int r_number = Robot_index;
    int r_number1;
    if(r_number == 1)r_number1 = 2;
    else r_number = 1;
    Vector3D ball = env.currentBall.pos;
    Vector3D robot = env.home[r_number].pos;
    Vector3D robot1 = env.home[r_number1].pos;


    double Robot_head_x = robot.x + half_robot*cos(env.home[r_number].rotation*deg2rad);
    double Robot_head_y = robot.y + half_robot*sin(env.home[r_number].rotation*deg2rad);
    double Robot1_head_x = robot1.x + half_robot*cos(env.home[r_number1].rotation*deg2rad);
    double Robot1_head_y = robot1.y + half_robot*sin(env.home[r_number1].rotation*deg2rad);


    double vector1br_x = ball.x - Robot1_head_x;
    double vector1br_y = ball.y - Robot1_head_y;

    double vectorbr_x = ball.x - Robot_head_x;
    double vectorbr_y = ball.y - Robot_head_y;


    double vectorrr_x = Robot1_head_x - Robot_head_x;
    double vectorrr_y = Robot1_head_y - Robot_head_y;

    double dis_br = hypot(vectorbr_x,vectorbr_y);
    double dis_rr = hypot(vectorrr_x,vectorrr_y);

    double v_x = vectorrr_x *4;
    double v_y = vectorrr_y *4;

    double phi = atan2(vectorrr_y , vectorrr_x) * rad2deg;
    double angle = phi - env.home[r_number].rotation;

    if(angle>180)
    {
        angle = angle - 360;
    }else if(angle < -180){
        angle = angle + 360;
    }
    if((fabs(robot.y) >= 2 && dis_br<=1)){
        env.home[1].v_x=v_x;
        env.home[1].v_y=v_y;
        env.home[1].v_yaw=angle;
    }else{
        strategy_Support(Robot_index);
    }
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
}

void FIRA_pathplan_class::strategy_AvoidBarrier(int Robot_index){

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
    if(n->getParam("/FIRA/Chase_Strategy", Chase_Strategy)){
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
   if(n->getParam("/FIRA/SideSpeedUp", SideSpeedUp)){
//       for(int i=0;i<5;i++)
//           std::cout<< "param SideSpeedUp["<< i << "]=" << SideSpeedUp[i] << std::endl;
//   std::cout << "====================================" << std::endl;
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
