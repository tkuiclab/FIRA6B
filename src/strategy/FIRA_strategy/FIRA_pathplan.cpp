#include "FIRA_pathplan.h"
#include "math.h"



FIRA_pathplan_class::FIRA_pathplan_class(){
   opponent = false;


}

//start---simulator---
void FIRA_pathplan_class::setEnv(Environment iEnv){
//void FIRA_pathplan_class::setEnv(){
   //Vector3D ball = iEnv.currentBall.pos;
   //printf("global_env   b_x=%lf,b_y=%lf\n",ball.x,ball.y);
   env = iEnv;

   if(opponent){
       for(int i = 0;i < PLAYERS_PER_SIDE;i++){
           env.home[i] = iEnv.opponent[i];
           env.opponent[i] = iEnv.home[i];
       }

   }
}
//end  ---simulator---

//車頭追球
void FIRA_pathplan_class::strategy_head2ball(int i)
{

   Vector3D ball = env.currentBall.pos;
   Vector3D robot = env.home[i].pos;
  // double robot_rotation = env.home[0].v_yaw;
   double robot_rotation = env.home[i].rotation;



   double x = ball.x - robot.x;
   if (x==0) x= very_small;
   double y = ball.y - robot.y;
   double a = atan2(y,x)*rad2deg; //*/

   //double a = 120;
   //if(x<0)       a = a-180;
   double        angle = a - robot_rotation;

   if(angle<-180)angle = angle+360;
   if(angle>180)angle = angle-360;


   env.home[i].v_yaw = angle*5;
}





//跑定點
void FIRA_pathplan_class::strategy_dst(double target_x,double target_y){

   //printf("dstX=%lf,dstY=%lf\n",dstX,dstY);

   double v_x = target_x - env.home[0].pos.x;
   double v_y = target_y - env.home[0].pos.y;

   env.home[0].v_x = v_x;
   env.home[0].v_y = v_y;
       //env.home[0].v_yaw = angle;
}

//車頭朝球＋走定點
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

   //v = 0.2;//1 * (1-exp(-distance/3));
   //a = atan(y/x) / pi * 180;
   double a = atan2(y_ball,x_ball) * rad2deg;
   //if(x_ball<0) a = a - 180;  //??
   angle = a - robot_rotation ;

   if(angle<-180) angle += 360;
   if(angle>=180) angle -= 360;

   //if(fabs(angle)==0)angle = very_small;

   //---speed planning---
   if(tar_dis < 0.1){
       v=0;
       if(fabs(angle) < 5) w=0;
       else w = angle/fabs(angle) * 200 * (1-exp(-1*fabs(angle)/10))+10;  //angular velocity
   }else{
       v = 250 * (1-exp(-1*tar_dis/10)) +10; //velocity
       w = 0;
   }




   //qDebug()<<angle;
   //qDebug()<<v;
//    env.home[0].velocityLeft =  (-(angle)/170+v)*10.0;
//    env.home[0].velocityRight = ((angle)/170+v)*10.0;


   //---command---
//    env.home[0].v_x = v*cos((robot_rotation+tar_ang)*deg2rad)/100;
//    env.home[0].v_y = v*sin((robot_rotation+tar_ang)*deg2rad)/100;
//    env.home[0].v_yaw = w/100;

   //---debug info---
   //printf("%f, %f, %f, %f, %f, %f\n",env.home[0].v_x,env.home[0].v_y,tar_dis,tar_ang,robot_rotation,angle);

       env.home[2].v_x = x;
       env.home[2].v_y = y;
       env.home[2].v_yaw = w;//angle;

}

void FIRA_pathplan_class::personalStrategy(int robotIndex,int action){

//    for(int i=0; i<PLAYERS_PER_SIDE;i++){
        switch(action){
            case action_Goalkeeper:
                strategy_Goalkeeper(robotIndex);
                //printf("goalkeeper  %d\n",robotIndex);
                break;
            case action_Attack:
//              strategy_dst_head2ball(-3,0);
                strategy_Attack(robotIndex);
//                strategy_typeS_Attack(robotIndex);
               //strategy_typeU_Attack(robotIndex);
                break;
            case action_typeS_attack:
                strategy_typeS_Attack(robotIndex);
                break;
            case action_Chase:
                strategy_Chase(robotIndex);
                break;
            case action_typeU_Attack:
                strategy_typeU_Attack(robotIndex);
                break;
            case action_Support:
                strategy_Support(robotIndex);
                //printf("support   %d\n",robotIndex);
                break;
            case action_AvoidBarrier:
                strategy_AvoidBarrier(robotIndex);
                break;
//            case Role_Halt:
//                strategy_Halt(robotIndex);
//                break;
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

//void FIRA_pathplan_class::strategy()
//{
//    //teamStrategy();

//    personalStrategy();

//}

void FIRA_pathplan_class::strategy_Goalkeeper(int Robot_index){

//    Vector3D ball = env.currentBall.pos;
//    Vector3D robot = env.home[Robot_index].pos;
//    Vector3D goal;

//    if(mTeam == Team_Blue)  {
//        goal = env.blue.pos;
////        printf("goal keeper say teamColor=Blue\n");

//    }else  {

//        goal = env.yellow.pos;
////        printf("goal keeper say teamColor=Yellow\n");
//    }

//    //printf("goal keeper say goal.x=%lf,goal.y = %lf\n",goal.x,goal.y);
////    printf("GoalKeeper say Robot_index=%d\n",Robot_index);


////    double head_x = robot.x + 0.29*cos(env.home[Robot_index].rotation*deg2rad);
////    double head_y = robot.y + 0.29*sin(env.home[Robot_index].rotation*deg2rad);
//    double vectorbr_x = ball.x - robot.x;
//    double vectorbr_y = ball.y - robot.y;
//    double vectordr_x = goal.x - robot.x;
//    double vectordr_y = goal.y - robot.y;
//    double vectorbd_x = ball.x - goal.x;
//    double vectorbd_y = ball.y - goal.y;
//    double distance_dr = hypot(vectordr_x,vectordr_y);
//    double distance_br = hypot(vectorbr_x,vectorbr_y);
//    double distance_bd = hypot(vectorbd_x,vectorbd_y);
//    double angle_bd = atan2(vectorbd_y , vectorbd_x) * rad2deg;
////    angle_br -= 180;
////    if (angle_bd!=angle_bd)angle_bd = 1;

//    double dst_x;
//    if(mTeam == Team_Blue){
//        dst_x =0.5/*0.5*/*cos(-angle_bd*deg2rad)+3-robot.x;
//    }else{
//        dst_x =goalkeeper_radius*cos(-angle_bd*deg2rad)-3-robot.x;
//    }
//    double dst_y =0.5*sin(angle_bd*deg2rad)-robot.y;

////    printf("dst_x= %lf\tdst_y = %lf\t_angle_bd = %lf\n",0.5*cos(-angle_bd*deg2rad)+3-robot.x,0.5*sin(angle_bd*deg2rad)-robot.y,angle_bd);
////    if(dst_x != dst_x) dst_x = very_small;
////    if(dst_y != dst_y) dst_y = very_small;

//    strategy_head2ball(Robot_index);

//    double a = vectorbr_x/distance_br;
//    double b = vectorbr_y/distance_br;
//    if(a != a)a = very_small;
//    if(b != b)b = very_small;

//    double v_x = ((a) + (vectordr_x/distance_dr)+dst_x)*2;
//    double v_y = ((b) + (vectordr_y/distance_dr)+dst_y)*2;

//    env.home[Robot_index].v_x =  v_x;
//    env.home[Robot_index].v_y =  v_y;

////    env.home[Robot_index].v_x =  ((vectorbr_x/distance_br) + (vectordr_x/distance_dr)+dst_x)*2;
////    env.home[Robot_index].v_y = (vectorbr_y/distance_br +vectordr_y/distance_dr+dst_y)*2;

//  //  printf("vectorbr_x= %lf\tvectordr_x = %lf\tvectorbr_y = %lf\tvectordr_y = %lf\n",vectorbr_x,vectordr_x,vectorbr_y,vectordr_y);
////    printf("_x= %lf\t_Y = %lf\t_angle = %lf\n",(vectorbr_x/distance_br + vectordr_x/distance_dr+dst_x)*2,(vectorbr_y/distance_br +vectordr_y/distance_dr+dst_y)*2,angle_bd);
////    printf("_x= %lf\t_Y = %lf\t_angle = %lf\n",cos(angle_bd*deg2rad),sin(angle_bd),angle_bd);

//    if(distance_bd<=1.75 && fabs(angle_bd)>=8 && fabs(angle_bd)<60){
//        env.home[Robot_index].v_x = vectorbr_x*5;
//        env.home[Robot_index].v_y = vectorbr_y*5;
//    }else if(distance_bd<=2 && fabs(angle_bd)<30){

//        env.home[Robot_index].v_x = vectorbr_x*8;
//        env.home[Robot_index].v_y = vectorbr_y*8;
//    }else if(distance_bd<=1.5 && fabs(angle_bd)>=60){

//        env.home[Robot_index].v_x = vectorbr_x*4;
//        env.home[Robot_index].v_y = vectorbr_y*4;
//    }

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

    double goal_dis = env.home[r_number].goal.distance;
    double goal_angle = env.home[r_number].goal.angle;
    double opgoal_dis = env.home[r_number].op_goal.distance;
    double opgoal_angle = env.home[r_number].op_goal.angle;

    double ball_x = -c_ball_dis*sin(c_ball_angle*deg2rad);
    double ball_y = c_ball_dis*cos(c_ball_angle*deg2rad);
    double goal_x = -goal_dis*sin(goal_angle*deg2rad);
    double goal_y = goal_dis*cos(goal_angle*deg2rad);

    double alpha = goal_angle - c_ball_angle;
    double beta = atan2( 0.73 , c_ball_dis) * rad2deg;

    Vector2d vectorbr(ball_x, ball_y);

    if(beta < alpha){
        alpha = beta;
    }else if(alpha < -beta){
        alpha = -beta;
    }

    double rotAngle = -alpha;

    Rotation2Dd rot( rotAngle * deg2rad);
    Vector2d vectornt = rot * vectorbr;

 //    printf("next target_x = %lf\tnext target_y =%lf\t",vectornt(0),vectornt(1));
 //    printf("alpha = %lf\tbeta = %lf\t",alpha, beta);
 //    printf("vectorbr_0 = %lf\tvectorbr_1 = %lf\n",vectorbr(0),vectorbr(1));

        if(fabs(alpha<=5) && c_ball_dis<=0.5) vectornt *= 4; /*3*///vectornt*(distance_bd/2+4);
        else if(goal_dis <= 2.5 && fabs(alpha<=30) && c_ball_dis<=0.5) vectornt *= 5;

        if(c_ball_dis <= 1) vectornt = vectornt * 4;
        else if (c_ball_dis > 1)vectornt = vectornt * 1;

//        printf("%lf  %lf\n", goal_angle, ball_angle);

        env.home[r_number].v_x =vectornt(0);
        env.home[r_number].v_y =vectornt(1);
        env.home[r_number].v_yaw = goal_angle*2;
//          env.home[r_number].v_x =0;
//          env.home[r_number].v_y =0;
//          env.home[r_number].v_yaw = 0;
}

void FIRA_pathplan_class::strategy_Attack(int Robot_index){

    double angle;
    //for(int r_number = 0; r_number <PLAYERS_PER_SIDE; r_number++){
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
    double opgoal_dis = env.home[r_number].op_goal.distance;
    double opgoal_angle = env.home[r_number].op_goal.angle;

//    printf("ball_dis=%lf, ball_angle=%lf,c_ball_dis=%lf, c_ball_angle=%lf\n",ball_dis, ball_angle,c_ball_dis,c_ball_angle);

//    double Robot_head_x = robot.x + half_robot*cos(env.home[r_number].rotation*deg2rad);
//    double Robot_head_y = robot.y + half_robot*sin(env.home[r_number].rotation*deg2rad);

 //   double Ball_tail_x = ball.x + 0.05* * cos();

    double ball_x = -c_ball_dis*sin(c_ball_angle*deg2rad);
    double ball_y = c_ball_dis*cos(c_ball_angle*deg2rad);
    double goal_x = -goal_dis*sin(goal_angle*deg2rad);
    double goal_y = goal_dis*cos(goal_angle*deg2rad);

//    double distance_bd = hypot(vectorbd_x,vectorbd_y);
//    double distance_a  = hypot(distance_br,1);

 //    printf("distance_br = %lf\tdistance_dr = %lf\t",distance_br, distance_dr);
    double alpha = goal_angle - c_ball_angle;
    double beta = atan2( 0.73 , c_ball_dis) * rad2deg;

//    Vector2d vectorbr(ball_x, ball_y);
    Vector2d vectornt(-1.42*sin(c_ball_angle*deg2rad),1.42*cos(c_ball_angle*deg2rad));

//    if(beta < alpha){
//        alpha = beta;
//    }else if(alpha < -beta){
//        alpha = -beta;
//    }

//    double rotAngle = -alpha;

//    Rotation2Dd rot( rotAngle * deg2rad);
//    Vector2d vectornt = rot * vectorbr;

 //    printf("next target_x = %lf\tnext target_y =%lf\t",vectornt(0),vectornt(1));
 //    printf("alpha = %lf\tbeta = %lf\t",alpha, beta);
 //    printf("vectorbr_0 = %lf\tvectorbr_1 = %lf\n",vectorbr(0),vectorbr(1));

//        if(fabs(alpha<=5) && c_ball_dis<=0.5) vectornt *= 4; /*3*///vectornt*(distance_bd/2+4);
//        else if(goal_dis <= 2.5 && fabs(alpha<=30) && c_ball_dis<=0.5) vectornt *= 5;

//        if(c_ball_dis <= 1) vectornt = vectornt * 4;
//        else if (c_ball_dis > 1)vectornt = vectornt * 1;

//        printf("%lf  %lf\n", goal_angle, ball_angle);

        env.home[r_number].v_x =vectornt(0);
        env.home[r_number].v_y =vectornt(1);
        env.home[r_number].v_yaw = goal_angle*2;
//    env.home[r_number].v_x =0;
//    env.home[r_number].v_y =0;
//    env.home[r_number].v_yaw = 0;
//        printf("v_x = %lf,v_y = %lf\n", vectornt(0),vectornt(1));
}

void FIRA_pathplan_class::strategy_Zone_Attack(int Robot_index){

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
    Vector2d vectordr(vectordr_x, vectordr_y);

    if(beta < angle_bd){
        angle_bd = beta;
    }else if(angle_bd < -beta){
        angle_bd = -beta;
    }

    double rotAngle = -angle_bd;

    Rotation2Dd rot( rotAngle * deg2rad);
    Vector2d vectornt = rot * vectorbr;

//--------------------------------------------------------------------
    if(fabs(angle_bd <= 5) && c_distance_br <= 0.5) vectornt *= 4; /*3*///vectornt*(distance_bd/2+4);
    else if(distance_dr <= 2.5 && fabs(angle_bd <= 30) && c_distance_br <= 0.5) vectornt *= 5;

    if(c_distance_br <= 1) vectornt = vectornt * 4;
    else if (c_distance_br > 1)vectornt = vectornt * 1;

    static int count1 = 0;
    static bool decide = 0;

    if(distance_dr < 1 && distance_dr > 0){
         decide=1;
    }
    if(decide == 1){
         vectornt=-(vectornt * 1);
         count1++;
         if(count1 >= 30){
             count1=0;
             decide=0;
         }
    }

    env.home[r_number].v_x =vectornt(0);
    env.home[r_number].v_y =vectornt(1);
    env.home[r_number].v_yaw = angle_dr * 2/*5*/;

}

void FIRA_pathplan_class::strategy_typeS_Attack(int Robot_index){

    int r_number = Robot_index; 	// assign which robot you are going to use

    double distance_br = env.home[r_number].ball.distance;	// distance between the ball and the robot
    double distance_dr = env.home[r_number].goal.distance;	// angle between robot head's direction and the ball

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
    double vectordr_x = -distance_dr * sin(angle_dr * deg2rad);
    double vectordr_y = distance_dr * cos(angle_dr * deg2rad);

    Vector2d vectorbr(vectorbr_x, vectorbr_y);
    Vector2d vectordr(vectordr_x, vectordr_y);

    if(beta < angle_bd){    // compare which angle we are going to use
        angle_bd = beta;
    }else if(angle_bd < -beta){
        angle_bd = -beta;
    }

    double rotAngle = -angle_bd;

    Rotation2Dd rot( rotAngle * deg2rad);
    Vector2d vectornt = rot * vectorbr;

    if(fabs(angle_bd <= 5) && c_distance_br <= 0.5) vectornt *= 4; /*3*///vectornt*(distance_bd/2+4);
    else if(distance_dr <= 2.5 && fabs(angle_bd <= 30) && c_distance_br <= 0.5) vectornt *= 5;

    if(c_distance_br <= 1) vectornt = vectornt * 4; // rush the door
    else if (c_distance_br > 1)vectornt = vectornt * 1;

    /*TYPE S attack */
    static int iCounter = 0;    // counter to set the process time
    static int iCondition = 0;  // condition when the ball is on the right's field and the left's

    double op_angle_dr = env.home[r_number].op_goal.angle;
    double dG2GAngle = angle_dr + op_angle_dr;  // angle to decide robot is on the right or left field

    if(distance_br < 0.35 && distance_br > 0 && fabs(angle_bd) < 8 && iCounter == 0){
        // condition when the robot is holding the ball and facing the door
        if(dG2GAngle >= 0){
            iCondition = 1;
        }else if(dG2GAngle < 0){
            iCondition = 2;
        }
    }

    if(iCondition == 1 && distance_dr > 1){   // on the right field
        iCounter++;
        if(iCounter > 50){  // reset the counter and the condition
            iCounter = 0;
            iCondition = 0;
            vectornt = rot * vectorbr;
        }else if(iCounter > 25){
            vectornt(0) = sin(iCounter * 2 * deg2rad) * 3;
        }else{  // go left first
            vectornt(0) = (-sin(iCounter * 2 * deg2rad)) * 3;
        }
    }else if(iCondition == 2 && distance_dr > 1){ // on the left field
        iCounter++;
        if(iCounter > 50){
            iCounter = 0;
            iCondition = 0;
//            vectornt = rot * vectorbr;
        }else if(iCounter > 25){
            vectornt(0) = (-sin(iCounter * 2 * deg2rad)) * 3;
        }else{  // go right first
            vectornt(0) = sin(iCounter * 2 * deg2rad) * 3;
        }
    }
    env.home[r_number].v_x = vectornt(0);
    env.home[r_number].v_y = vectornt(1);
    env.home[r_number].v_yaw = angle_dr * 2/*5*/;
//    printf("iCondition = %d\t iCounter = %d\n", iCondition, iCounter);
//    printf("distance_br = %lf\t op_angle_dr = %lf\n", distance_br, op_angle_dr);
}
void FIRA_pathplan_class::strategy_typeU_Attack(int Robot_index){
    int r_number = Robot_index;
    
    //Vector3D blueGoal = env.yellow.pos;
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
    
    //   printf("distance_br = %lf\tdistance_dr = %lf\t",distance_br, distance_dr);
    
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
    double dG2GAngle = angle_dr + op_angle_dr;  // angle between the robot to our goal and opponent's
    
    static int id=0;
    static int iCondition=0;
    
    
    if(fabs(c_angle_br) <= 10){
        if(id != 1){
            if(dG2GAngle >= 0){
                iCondition = 1;
            }else if(dG2GAngle < 0){
                iCondition = 2;
            }
        }
    }
    
    
    if(iCondition == 1){
        id=1;
        if(c_distance_br >0.25){                       //chase ball
            env.home[r_number].v_x = vectorbr_x;
            env.home[r_number].v_y = vectorbr_y;
            env.home[r_number].v_yaw = angle_br;
        }else if(c_distance_br <= 0.25 ){
            env.home[r_number].v_y = 1;             //change cycle big or small
            env.home[r_number].v_yaw =90;           //change cycle big or small
            if(fabs(angle_dr) <= 15 && c_distance_br <= 0.5){       //chase goal
                vectornt *= 10;
                env.home[r_number].v_x = vectornt(0);
                env.home[r_number].v_y = vectornt(1);
                env.home[r_number].v_yaw = angle_dr * 3;
                if(distance_bd < 0.85){
                    id=0;
                }
            }
        }
    }else if(iCondition == 2){
        id=1;
        if(c_distance_br >0.25){                       //chase ball
            env.home[r_number].v_x = vectorbr_x;
            env.home[r_number].v_y = vectorbr_y;
            env.home[r_number].v_yaw = angle_br ;
        }else if(c_distance_br <= 0.25){
            env.home[r_number].v_y = 1;             //change cycle big or small
            env.home[r_number].v_yaw =-90;           //change cycle big or small
            if(fabs(angle_dr) <=15 && c_distance_br <= 0.5){       //chase goal
                vectornt *= 10;
                env.home[r_number].v_x = vectornt(0);
                env.home[r_number].v_y = vectornt(1);
                env.home[r_number].v_yaw = angle_dr * 3;
                if(distance_bd < 0.85){
                    id=0;
                }
            }
        }
    }else{
        env.home[r_number].v_x = vectorbr_x;
        env.home[r_number].v_y = vectorbr_y;
        env.home[r_number].v_yaw = angle_br ;
    }    
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

 //    printf("next target_x = %lf\tnext target_y =%lf\t",vectornt(0),vectornt(1));
 //    printf("alpha = %lf\tbeta = %lf\t",alpha, beta);
 //    printf("vectorbr_0 = %lf\tvectorbr_1 = %lf\n",vectorbr(0),vectorbr(1));


 ///-----------------------車頭--------------------------
        angle = phi - env.home[r_number].rotation;
        if(angle>180)
        {
            angle = angle - 360;
        }else if(angle < -180){
            angle = angle + 360;
        }

//        if(fabs(alpha<=long_rush_alpha) && dis_br<=long_rush_dis_br) vectornt *= long_rush_speed_const; /*3*///vectornt*(distance_bd/2+4);
//        else if(distance_dr <= short_rush_dis_dr/*2.5*/ && fabs(alpha<=short_rush_alpha/*30*/) && dis_br<=short_rush_dis_br /*0.5*/) vectornt *= short_rush_speed_const/*4*/;

//        if(dis_br <= close_ball_dis_const) vectornt = vectornt * close_ball_speed_const;
//        else if (dis_br > close_ball_dis_const)vectornt = vectornt * far_ball_speed_const;
 //printf("%lf\n", alpha);
//        printf("v_x = %lf,v_y = %lf\n", vectornt(0),vectornt(1));
        env.home[r_number].v_x = vectornt(0);
        env.home[r_number].v_y = vectornt(1);
        env.home[r_number].v_yaw = angle*5/*5*/;
//        strategy_dontboom();

}

void FIRA_pathplan_class::strategy_Halt(int Robot_index){

    env.home[Robot_index].v_x = 0;
    env.home[Robot_index].v_y = 0;
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
//    strategy_dontboom();
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
//    strategy_dontboom();
}

void FIRA_pathplan_class::strategy_CornerKick(int Robot_index){

    int r_number = Robot_index;   // assign which robot you are going to use

    double distance_br = env.home[r_number].ball.distance;  // distance between the ball and the robot
    double distance_dr = env.home[r_number].goal.distance;  // angle between robot head's direction and the ball

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
    double vectordr_x = -distance_dr * sin(angle_dr * deg2rad);
    double vectordr_y = distance_dr * cos(angle_dr * deg2rad);

    Vector2d vectorbr(vectorbr_x, vectorbr_y);
    Vector2d vectordr(vectordr_x, vectordr_y);

    if(beta < angle_bd){
        angle_bd = beta;
    }else if(angle_bd < -beta){
        angle_bd = -beta;
    }

    double rotAngle = -angle_bd;

    Rotation2Dd rot( rotAngle * deg2rad);
    Vector2d vectornt = rot * vectorbr;

    if(fabs(angle_bd <= 5) && c_distance_br <= 0.5) vectornt *= 4; /*3*///vectornt*(distance_bd/2+4);
    else if(distance_dr <= 2.5 && fabs(angle_bd <= 30) && c_distance_br <= 0.5) vectornt *= 5;

    if(c_distance_br <= 1) vectornt = vectornt * 4; // rush the door
    else if (c_distance_br > 1)vectornt = vectornt * 1;

    /*------------------------Corner Kick---------------------------*/
    double op_angle_dr = env.home[r_number].op_goal.angle;
    const int THRESHOLD = 300;
    static int iCounter = 0;
    static int iCondition = 0;  // condition when the ball is on the right's field or the left's

    /* use y (forward) velocity and the robot's target angle to make the robot go in a curve */

    if(op_angle_dr > 0 && iCounter == 0){   // robot kicks on the right field
        iCondition = 1;
    }else if(op_angle_dr < 0 && iCounter == 0){ // robot kicks on the left field
        iCondition = 2;
    }

    if(distance_dr < 1){  // if goal then reset the counter and condition
        iCondition = 0;
        iCounter = 0;
    }

    if(iCondition == 1 && fabs(angle_bd) > 10 && iCounter < THRESHOLD){
        if(iCounter > THRESHOLD * 1 / 3){   // timing to enlarge angle in order to face the attacking door
            env.home[r_number].v_yaw = 55;
        }else{
            env.home[r_number].v_yaw = 20;
        }
        iCounter++;
        env.home[r_number].v_y = 1.2;
    }else if(iCondition == 2 && fabs(angle_bd) > 10 && iCounter < THRESHOLD){
        if(iCounter > THRESHOLD  * 1 / 3){
            env.home[r_number].v_yaw = -55;
        }else{
            env.home[r_number].v_yaw = -20;
        }
        iCounter++;
        env.home[r_number].v_y = 1.2;
    }else if(iCounter > 0){ // rush to the door
        env.home[r_number].v_x = vectornt(0);
        env.home[r_number].v_y = vectornt(1);
        env.home[r_number].v_yaw = angle_dr * 2;
    }

//    printf("iCounter = %d\t v_yaw = %lf\t %lf\n", iCounter, env.home[r_number].v_yaw, distance_dr);
    /*--------------------------------------------------------------*/

}
void FIRA_pathplan_class::strategy_AvoidBarrier(int Robot_index){

}
//------------------------------------------strategy choose end----------------

//void FIRA_pathplan_class::strategy_dontboom(){
//        Vector3D ball = env.currentBall.pos;
//        Vector3D robot = env.home[0].pos;
//        Vector3D robot1 = env.home[1].pos;

//        double vectorbr_x = ball.x - robot.x;
//        double vectorbr_y = ball.y - robot.y;

//        double vector1br_x = ball.x - robot1.x;
//        double vector1br_y = ball.y - robot1.y;

//        double distance_rr = hypot(robot.x - robot1.x, robot.y - robot1.y);
//        double distance_br = hypot(vectorbr_x,vectorbr_y);

//        double distance1_br = hypot(vector1br_x,vector1br_y);

//        if(distance_rr<=0.9 && distance_br >= distance1_br){
//            env.home[0].v_x = 0;
//            env.home[0].v_y = 0;
//        }else if(distance_rr<=0.9 && distance_br < distance1_br){
//            env.home[1].v_x = 0;
//            env.home[1].v_y = 0;
//        }
//}
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

void FIRA_pathplan_class::loadParam(ros::NodeHandle *n){

    std::string ns = "/FIRA/R1/Strategy/Pathplan/";

    if(n->getParam("/FIRA/TeamColor",teamColor)){
        std::cout << "param teamColor=" << teamColor <<std::endl;
    }
    if(n->getParam(ns + "beta_const",beta_const)){
        std::cout << "beta_const=" << beta_const <<std::endl;
    }
    if(n->getParam(ns + "long_rush_alpha",long_rush_alpha)){
        std::cout << "long_rush_alpha=" << long_rush_alpha <<std::endl;
    }
    if(n->getParam(ns + "long_rush_speed_const",long_rush_speed_const)){
        std::cout << "long_rush_speed_const=" << long_rush_speed_const <<std::endl;
    }
    if(n->getParam(ns + "long_rush_dis_br",long_rush_dis_br)){
        std::cout << "long_rush_dis_br=" << long_rush_dis_br <<std::endl;
    }
    if(n->getParam(ns + "short_rush_dis_dr",short_rush_dis_dr)){
        std::cout << "short_rush_dis_dr=" << short_rush_dis_dr <<std::endl;
    }
    if(n->getParam(ns + "short_rush_alpha",short_rush_alpha)){
        std::cout << "short_rush_alpha=" << short_rush_alpha <<std::endl;
    }
    if(n->getParam(ns + "short_rush_dis_br",short_rush_dis_br)){
        std::cout << "short_rush_dis_br=" << short_rush_dis_br <<std::endl;
    }
    if(n->getParam(ns + "short_rush_speed_const",short_rush_speed_const)){
        std::cout << "short_rush_speed_const=" << short_rush_speed_const <<std::endl;
    }
    if(n->getParam(ns + "close_ball_speed_const",close_ball_speed_const)){
        std::cout << "close_ball_speed_const=" << close_ball_speed_const <<std::endl;
    }
    if(n->getParam(ns + "close_ball_dis_const",close_ball_dis_const)){
        std::cout << "close_ball_dis_const=" << close_ball_dis_const <<std::endl;
    }
    if(n->getParam(ns + "far_ball_speed_const",far_ball_speed_const)){
        std::cout << "far_ball_speed_const=" << far_ball_speed_const <<std::endl;
    }
    if(n->getParam(ns + "head2ball_speed",head2ball_speed)){
        std::cout << "head2ball_speed=" << head2ball_speed <<std::endl;
    }
    //------------------------GOALKEEPER_CONST------------------------------------
    if(n->getParam(ns + "goalkeeper_radius",goalkeeper_radius)){
        std::cout << "goalkeeper_radius=" << goalkeeper_radius <<std::endl;
    }
    if(n->getParam(ns + "goalkeeper_front_dis",goalkeeper_front_dis)){
        std::cout << "goalkeeper_front_dis=" << goalkeeper_front_dis <<std::endl;
    }
    if(n->getParam(ns + "goalkeeper_mid_dis",goalkeeper_mid_dis)){
        std::cout << "goalkeeper_mid_dis=" << goalkeeper_mid_dis <<std::endl;
    }
    if(n->getParam(ns + "goalkeeper_side_dis",goalkeeper_side_dis)){
        std::cout << "goalkeeper_side_dis=" << goalkeeper_side_dis <<std::endl;
    }
    if(n->getParam(ns + "goalkeeper_front_angle",goalkeeper_front_angle)){
        std::cout << "goalkeeper_front_angle=" << goalkeeper_front_angle <<std::endl;
    }
    if(n->getParam(ns + "goalkeeper_mid_angle",goalkeeper_mid_angle)){
        std::cout << "goalkeeper_mid_angle=" << goalkeeper_mid_angle <<std::endl;
    }
    if(n->getParam(ns + "goalkeeper_front_speed",goalkeeper_front_speed)){
        std::cout << "goalkeeper_front_speed=" << goalkeeper_front_speed <<std::endl;
    }
    if(n->getParam(ns + "goalkeeper_mid_speed",goalkeeper_mid_speed)){
        std::cout << "goalkeeper_mid_speed=" << goalkeeper_mid_speed <<std::endl;
    }
    if(n->getParam(ns + "goalkeeper_side_speed",goalkeeper_side_speed)){
        std::cout << "goalkeeper_side_speed=" << goalkeeper_side_speed <<std::endl;
    }

}
