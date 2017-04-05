#include "teamStrategy_nodeHandle.h"

TeamStrategy_nodeHandle::
TeamStrategy_nodeHandle(int argc, char** argv):
BaseNode(argc,argv,Node_Name)
{
    
}

void TeamStrategy_nodeHandle::ros_comms_init(){
    n = new ros::NodeHandle();
    //prefix
    std::string robot_prefix = Robot_Topic_Prefix;
    std::string robotOpt_prefix = RobotOpt_Topic_Prefix;
    //suffix
    std::string robotPos_suffix = Robot_Position_Topic_Suffix;
    
    Gazebo_Model_Name_sub = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&TeamStrategy_nodeHandle::find_gazebo_model_name_fun,this);
    
    ball_sub = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&TeamStrategy_nodeHandle::ball_sub_fun,this);
    
    robot_1_pos_sub   = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&TeamStrategy_nodeHandle::robot_1_pos_fun,this);
    robot_2_pos_sub   = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&TeamStrategy_nodeHandle::robot_2_pos_fun,this);
    robot_3_pos_sub   = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&TeamStrategy_nodeHandle::robot_3_pos_fun,this);

    robotOpt_1_pos_sub = n->subscribe<nav_msgs::Odometry>(robotOpt_prefix  +"1"+robotPos_suffix,1000,&TeamStrategy_nodeHandle::robotOpt_1_pos_fun,this);
    robotOpt_2_pos_sub = n->subscribe<nav_msgs::Odometry>(robotOpt_prefix  +"2"+robotPos_suffix,1000,&TeamStrategy_nodeHandle::robotOpt_2_pos_fun,this);
    robotOpt_3_pos_sub = n->subscribe<nav_msgs::Odometry>(robotOpt_prefix  +"3"+robotPos_suffix,1000,&TeamStrategy_nodeHandle::robotOpt_3_pos_fun,this);
    
    GameState = n->subscribe<std_msgs::Int32>(GameState_Topic,1000,&TeamStrategy_nodeHandle::subGameState,this);
    TeamColor = n->subscribe<std_msgs::String>(TeamColor_Topic,1000,&TeamStrategy_nodeHandle::subTeamColor,this);

    //robot_role
    std::string robot_role_prefix = opponent ?  RobotOpt_Topic_Prefix : Robot_Topic_Prefix;
    std::string robot_role_suffix = Robot_Role_Topic_Suffix;
        
    robot_2_role_pub = n->advertise<std_msgs::Int32>(robot_role_prefix+"2"+robot_role_suffix,1000);
    robot_3_role_pub = n->advertise<std_msgs::Int32>(robot_role_prefix+"3"+robot_role_suffix,1000);    
}
void TeamStrategy_nodeHandle::Transfer(int r_number){
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
    }else if (Team_color == Team_Yellow){
        Goal = global_env->blue.pos;
        Goal_op = global_env->yellow.pos;
    }
    
    double Robot_head_x = Robot.x + half_robot*cos(global_env->home[r_number].rotation*deg2rad);
    double Robot_head_y = Robot.y + half_robot*sin(global_env->home[r_number].rotation*deg2rad);
    
    double vectorbr_x = Ball.x - Robot_head_x;
    double vectorbr_y = Ball.y - Robot_head_y;
    double vectordr_x = Goal.x - Robot_head_x;
    double vectordr_y = Goal.y - Robot_head_y;
    double vectordr_x_op = Goal.x - Robot_head_x;
    double vectordr_y_op = Goal.y - Robot_head_y;
    
    double phi_br = atan2(vectorbr_y, vectorbr_x) * rad2deg;
    double phi_dr = atan2(vectordr_y, vectordr_x) * rad2deg; //target deg
    double phi_dr_op = atan2(vectordr_y_op, vectordr_x_op) * rad2deg;
    /*----------------------The input value of the real robot------------------------------------------------*/
    global_env->home[r_number].ball.distance = hypot(vectorbr_x,vectorbr_y);
    global_env->home[r_number].goal.distance = hypot(vectordr_x,vectordr_y);
    global_env->home[r_number].op_goal.distance = hypot(vectordr_x_op, vectordr_y_op);
    
    global_env->home[r_number].goal.angle = phi_dr - global_env->home[r_number].rotation; // angle between robot to our attacking goal and robot's head direction
    global_env->home[r_number].ball.angle = phi_br - global_env->home[r_number].rotation; // angle between robot to the ball and robot's head direction
    global_env->home[r_number].op_goal.angle = phi_dr_op - global_env->home[r_number].rotation;// angle between robot to opponent's attacking goal and robot's head direction
}
