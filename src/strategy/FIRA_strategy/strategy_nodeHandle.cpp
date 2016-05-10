#include "strategy_nodeHandle.h"
#include "FIRA_pathplan.h"


Strategy_nodeHandle::
Strategy_nodeHandle(int argc, char** argv):
    BaseNode(argc,argv,Node_Name)
{
    roleAry[0] = Role_Goalkeeper;
}


void Strategy_nodeHandle::ros_comms_init(){
    n = new ros::NodeHandle();

    /*
    global_env->blue.pos.x = 3.35;
    global_env->blue.pos.y = 0;
    global_env->blue.pos.z = 0;

    global_env->yellow.pos.x = -3.35;
    global_env->yellow.pos.y = 0;
    global_env->yellow.pos.z = 0;
    */
    ////std::cout << "blue goal   x="<<global_env->blue.pos.x << ",y="<<global_env->blue.pos.y << std::endl;
    ////std::cout << "yellow goal   x="<<global_env->yellow.pos.x << ",y="<<global_env->yellow.pos.y << std::endl;


    //robotSpeedPub = n->advertise<FIRA_status_plugin::RobotSpeedMsg>(Pub_Speed_Topic,1000);

    std::string robot_prefix = Robot_Topic_Prefix;
    std::string robotOpt_prefix = RobotOpt_Topic_Prefix;
    std::string robotPos_suffix = Robot_Position_Topic_Suffix;

    //gazebo_ModelStates subscriber
    Gazebo_Model_Name_sub = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&Strategy_nodeHandle::find_gazebo_model_name_fun,this);

    //Use_topic_gazebo_msgs_Model_States to get model position
    ball_sub = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&Strategy_nodeHandle::ball_sub_fun,this);

    //robot subscriber
    robot_1_pos_sub   = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&Strategy_nodeHandle::robot_1_pos_fun,this);
    robot_2_pos_sub   = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&Strategy_nodeHandle::robot_2_pos_fun,this);
    robot_3_pos_sub   = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&Strategy_nodeHandle::robot_3_pos_fun,this);
    robotOpt_1_pos_sub = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&Strategy_nodeHandle::robotOpt_1_pos_fun,this);
    robotOpt_2_pos_sub = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&Strategy_nodeHandle::robotOpt_2_pos_fun,this);
    robotOpt_3_pos_sub = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&Strategy_nodeHandle::robotOpt_3_pos_fun,this);
    GameState = n->subscribe<std_msgs::Int32>(GameState_Topic,1000,&Strategy_nodeHandle::subGameState,this);
    TeamColor = n->subscribe<std_msgs::String>(TeamColor_Topic,1000,&Strategy_nodeHandle::subTeamColor,this);

    //ball subscriber
//    ball_sub = n->subscribe<FIRA_status_plugin::ModelMsg>(Ball_Topic_Name,1000,&Strategy_nodeHandle::ball_sub_fun,this);

//    //robot subscriber
//    robot_1_pos_sub   = n->subscribe<nav_msgs::Odometry>(robot_prefix  +"1"+robotPos_suffix,1000,&Strategy_nodeHandle::robot_1_pos_fun,this);
//    robot_2_pos_sub   = n->subscribe<nav_msgs::Odometry>(robot_prefix  +"2"+robotPos_suffix,1000,&Strategy_nodeHandle::robot_2_pos_fun,this);
//    robot_3_pos_sub   = n->subscribe<nav_msgs::Odometry>(robot_prefix  +"3"+robotPos_suffix,1000,&Strategy_nodeHandle::robot_3_pos_fun,this);
//    robotOpt_1_pos_sub = n->subscribe<nav_msgs::Odometry>(robotOpt_prefix  +"1"+robotPos_suffix,1000,&Strategy_nodeHandle::robotOpt_1_pos_fun,this);
//    robotOpt_2_pos_sub = n->subscribe<nav_msgs::Odometry>(robotOpt_prefix  +"2"+robotPos_suffix,1000,&Strategy_nodeHandle::robotOpt_2_pos_fun,this);
//    robotOpt_3_pos_sub = n->subscribe<nav_msgs::Odometry>(robotOpt_prefix  +"3"+robotPos_suffix,1000,&Strategy_nodeHandle::robotOpt_3_pos_fun,this);

    //robot_role
    std::cout << "Strategy_nodeHandle::ros_comms_init() say opponent = " << opponent << std::endl;
    std::string robot_role_prefix = opponent ?  RobotOpt_Topic_Prefix : Robot_Topic_Prefix;
    std::string robot_role_suffix = Robot_Role_Topic_Suffix;

    //role subscriber
    robot_2_role_sub = n->subscribe<std_msgs::Int32>(robot_role_prefix  +"2"+robot_role_suffix,1000,&Strategy_nodeHandle::robot_2_role_fun,this);
    robot_3_role_sub = n->subscribe<std_msgs::Int32>(robot_role_prefix  +"3"+robot_role_suffix,1000,&Strategy_nodeHandle::robot_3_role_fun,this);

    //speed subscriber
    std::string robotSpeed_suffix = RobotSpeed_Topic_Suffix;
    robot_1_speed_pub = n->advertise<geometry_msgs::Twist>(robot_prefix+"1"+robotSpeed_suffix,1000);
    robot_2_speed_pub = n->advertise<geometry_msgs::Twist>(robot_prefix+"2"+robotSpeed_suffix,1000);
    robot_3_speed_pub = n->advertise<geometry_msgs::Twist>(robot_prefix+"3"+robotSpeed_suffix,1000);
    robotOpt_1_speed_pub = n->advertise<geometry_msgs::Twist>(robotOpt_prefix+"1"+robotSpeed_suffix,1000);
    robotOpt_2_speed_pub = n->advertise<geometry_msgs::Twist>(robotOpt_prefix+"2"+robotSpeed_suffix,1000);
    robotOpt_3_speed_pub = n->advertise<geometry_msgs::Twist>(robotOpt_prefix+"3"+robotSpeed_suffix,1000);


    //std::cout << "PersonalStrategy ros_comms_init() finish" << std::endl;


}

void Strategy_nodeHandle::Transfer(int r_number){
    /* Transfer the value of the x, y axis of robot, ball and goal into the distance and angle
     * , in order to synchrnize the value of simulator and reality*/

/*-------------------The input value of the simulator -----------------------------------*/
    Vector3D Ball = global_env->currentBall.pos;
    Vector3D Robot = global_env->home[r_number].pos;
    Vector3D Goal;
    Vector3D Goal_op;

    int Team_color = Team_Blue;

    /* extinguish the team color and then set the goal */

    if(Team_color == Team_Blue){
        Goal = global_env->yellow.pos;
        Goal_op = global_env->blue.pos;
//        goal_color = Goal_Yellow;

    }else if (Team_color == Team_Yellow){
        Goal = global_env->blue.pos;
        Goal_op = global_env->yellow.pos;
//        goal_color = Goal_Blue;

    }

    double vectorbr_x = Ball.x - Robot.x;
    double vectorbr_y = Ball.y - Robot.y;
    double vectordr_x = Goal.x - Robot.x;
    double vectordr_y = Goal.y - Robot.y;
    double vectordr_x_op = Goal_op.x - Robot.x;
    double vectordr_y_op = Goal_op.y - Robot.y;

    double phi_br = atan2(vectorbr_y, vectorbr_x) * rad2deg;
    double phi_dr = atan2(vectordr_y, vectordr_x) * rad2deg; //target deg
    double phi_dr_op = atan2(vectordr_y_op, vectordr_x_op) * rad2deg;
/*----------------------The input value of the real robot------------------------------------------------*/
    global_env->home[r_number].ball.distance = hypot(vectorbr_x,vectorbr_y);
    global_env->home[r_number].goal.distance = hypot(vectordr_x,vectordr_y);
    global_env->home[r_number].op_goal.distance = hypot(vectordr_x_op, vectordr_y_op);

    double goal_angle = phi_dr - global_env->home[r_number].rotation;
    if(goal_angle > 180)
    {
        goal_angle = goal_angle - 360;
    }else if(goal_angle < -180){
        goal_angle = goal_angle + 360;
    }
    double ball_angle = phi_br - global_env->home[r_number].rotation;
    if(ball_angle > 180)
    {
        ball_angle = ball_angle - 360;
    }else if(ball_angle < -180){
        ball_angle = ball_angle + 360;
    }
    double op_goal_angle = phi_dr_op - global_env->home[r_number].rotation;;
    if(op_goal_angle > 180)
    {
        op_goal_angle = op_goal_angle - 360;
    }else if(op_goal_angle < -180){
        op_goal_angle = op_goal_angle + 360;
    }
    global_env->home[r_number].goal.angle = goal_angle; // angle between robot to our attacking goal and robot's head direction
    global_env->home[r_number].ball.angle = ball_angle; // angle between robot to the ball and robot's head direction
    global_env->home[r_number].op_goal.angle = op_goal_angle;// angle between robot to opponent's attacking goal and robot's head direction

}
void Strategy_nodeHandle::rotateXY(double rotate,double inX,double inY,double &newX,double &newY){
    double rad = rotate*deg2rad;
    newX = inX*cos(rad)+inY*sin(rad);
    newY = -inX*sin(rad)+inY*cos(rad);
}

void Strategy_nodeHandle::pubSpeed(ros::Publisher *puber,double v_x,double v_y,double v_yaw,double robot_rot){
    double r_v_x,r_v_y;
    rotateXY(robot_rot,v_x,v_y,r_v_x,r_v_y);//relatively 2 absolutely

    geometry_msgs::Twist speedMsg;
    //=====simulator use=====
    speedMsg.linear.x = /*r_*/v_y;
    speedMsg.linear.y = /*r_*/(-v_x);
    speedMsg.angular.z = v_yaw ;
//    speedMsg.linear.x = 0;
//    speedMsg.linear.y = 0;
    //=====entity use=====
//    speedMsg.linear.x = /*r_*/v_y;
//    speedMsg.linear.y = /*r_*/v_x;
//    speedMsg.angular.z = v_yaw ;
    double simulator = false;
    if(simulator)
      puber->publish(speedMsg);
    else{
        velocity_S_planning(&speedMsg);
        puber->publish(speedMsg);
    }
}

void Strategy_nodeHandle::velocity_S_planning(geometry_msgs::Twist *msg){
//    msg->linear.x = 0;
//    msg->linear.y=10;
//    msg->angular.z=0;
    double Vdis = hypot(msg->linear.x,msg->linear.y);
    double alpha = atan(-msg->linear.x/msg->linear.y)*rad2deg;
    if(msg->linear.y>0){
        if(msg->linear.x>0)
            alpha+=180;
        else
            alpha-=180;
    }
//    std::cout << "alpha="<<alpha<<std::endl;
    double angle = msg->angular.z;
    double Vdis_max = 2.5;
    double Vdis_min = 0.3;
    double VTdis_max = 80;
    double VTdis_min = 20;
    double VTdis;
    double Tangle_max = 80;
    double angle_max = Tangle_max*(9/5);
    double Tangle;
////Transfer vector to [0,100]
    if(Vdis>Vdis_max)
        VTdis=VTdis_max;
    else if(Vdis<Vdis_min)
        VTdis=VTdis_min;
    else
        VTdis = (VTdis_max-VTdis_min)*(cos(pi*((Vdis-Vdis_min)/(Vdis_max-Vdis_min)-1))+1)/2+VTdis_min;
////Transfer yaw to [0,100]
    if(fabs(angle)>angle_max)
        Tangle=angle_max;
    else
        Tangle=(Tangle_max-0)*(cos(pi*((fabs(angle)-0)/(angle_max-0)-1))+1)/2+0;
//// [-100,100]
    if(angle<0)
        Tangle = -Tangle;
        msg->linear.x = VTdis*sin(alpha*deg2rad);
        msg->linear.y = -VTdis*cos(alpha*deg2rad);
        msg->angular.z = Tangle;
//    std::cout<<"yaw = "<<msg->angular.z <<"\tmsg->linear.x ="<<msg->linear.x <<"\tmsg->linear.y ="<<-msg->linear.y <<std::endl;
}

void Strategy_nodeHandle::pubGrpSpeed(){
    ////--------------------speed test----------------
    double dVectorMax = VectorMax;
    double dVectorMin = VectorMin;
    double dVector[3];
    for(int i=0;i<3;i++){
        dVector[i] = hypot(global_env->home[i].v_x,global_env->home[i].v_y);
        double gama = atan2(global_env->home[i].v_y,global_env->home[i].v_x);
        if(dVector[i]>dVectorMax){
            c_v_x=VectorMax*cos(gama);
            c_v_y=VectorMax*sin(gama);

            global_env->home[i].v_x = c_v_x;
            global_env->home[i].v_y = c_v_y;
         }else if(dVector[i]<dVectorMin && dVector[i]>0){
            c_v_x=VectorMin*cos(gama);
            c_v_y=VectorMin*sin(gama);

            global_env->home[i].v_x = c_v_x;
            global_env->home[i].v_y = c_v_y;
        }
//        printf("%d. c_v_x=%lf, c_v_y=%lf ",i,c_v_x,c_v_y);
    }/*printf("\n");*/
////-------------------------pub----------------------------------------
    if(!opponent){
        pubSpeed(&robot_1_speed_pub  ,global_env->home[0].v_x,global_env->home[0].v_y,global_env->home[0].v_yaw,global_env->home[0].rotation);
        pubSpeed(&robot_2_speed_pub  ,global_env->home[1].v_x,global_env->home[1].v_y,global_env->home[1].v_yaw,global_env->home[1].rotation);
        pubSpeed(&robot_3_speed_pub  ,global_env->home[2].v_x,global_env->home[2].v_y,global_env->home[2].v_yaw,global_env->home[2].rotation);
    }else{
        pubSpeed(&robotOpt_1_speed_pub,global_env->opponent[0].v_x,global_env->opponent[0].v_y,global_env->opponent[0].v_yaw,global_env->opponent[0].rotation);
        pubSpeed(&robotOpt_2_speed_pub,global_env->opponent[1].v_x,global_env->opponent[1].v_y,global_env->opponent[1].v_yaw,global_env->opponent[1].rotation);
        pubSpeed(&robotOpt_3_speed_pub,global_env->opponent[2].v_x,global_env->opponent[2].v_y,global_env->opponent[2].v_yaw,global_env->opponent[2].rotation);
//            printf("1: %lf %lf %lf \n",global_env->opponent[0].v_x,global_env->opponent[0].v_y,global_env->opponent[0].v_yaw);
//            printf("2: %lf %lf %lf \n",global_env->opponent[1].v_x,global_env->opponent[1].v_y,global_env->opponent[1].v_yaw);
//            printf("3: %lf %lf %lf \n",global_env->opponent[2].v_x,global_env->opponent[2].v_y,global_env->opponent[2].v_yaw);
    }
}
