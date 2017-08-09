/**
 * @file BaseNode.h
 *
 * @brief Ros communication central!
 *
 * @date February 2014
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef TeamStrategy_nodeHandle_HPP_
#define TeamStrategy_nodeHandle_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include <ros/ros.h>
#include <string>
#include "../common/Env.h"
#include "../common/BaseNode.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "FIRA_status_plugin/RobotSpeedMsg.h"
#include "FIRA_status_plugin/ModelMsg.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"
#include "vision/Object.h"
#include "vision/Two_point.h"
/*****************************************************************************
 ** Define
 *****************************************************************************/
#define Ball_Topic_Name "/FIRA/Strategy/WorldMap/soccer"
#define GameState_Topic "/FIRA/GameState"

//RobotNumber
#define RobotNumber_Topic "/FIRA/RobotNumber"
#define TeamColor_Topic "/FIRA/TeamColor"
//robot prefix
#define Robot_Topic_Prefix "/FIRA/R"
#define RobotOpt_Topic_Prefix "/FIRA_Opt/R"

//robot suffix
#define Robot_Position_Topic_Suffix "/Strategy/WorldMap/RobotPos"
#define Robot_Role_Topic_Suffix "/Strategy/Coach/role"

#define Node_Name "TeamStrategy"

#define ModelState_Topic_Name  "/gazebo/model_states"
#define Vision_Topic "/vision/object"
#define Vision_Two_point_Topic "/interface/Two_point"
/*****************************************************************************
 ** Class
 *****************************************************************************/

class TeamStrategy_nodeHandle :public BaseNode {
public:
    TeamStrategy_nodeHandle(int argc, char** argv);
    
    virtual ~TeamStrategy_nodeHandle(){}
    void setEnv(Environment *inEnv){global_env = inEnv;}
    void setOpponent(bool inBool){opponent = inBool;}
    
    void pubRole(int *roleAry){
        std_msgs::Int32 robot_1_role;
        robot_1_role.data = roleAry[0]; //roleAry[1-1]
        robot_1_role_pub.publish(robot_1_role);

        std_msgs::Int32 robot_2_role;
        robot_2_role.data = roleAry[1]; //roleAry[2-1]
        robot_2_role_pub.publish(robot_2_role);
        
        
        std_msgs::Int32 robot_3_role;
        robot_3_role.data = roleAry[2];  //roleAry[3-1]
        robot_3_role_pub.publish(robot_3_role);
    }
    
    
    ros::NodeHandle* getNodeHandle(){return n;}
    
protected:
    void ros_comms_init();
    
private:
    
    Environment *global_env;
    bool opponent;
    bool run_one = false;
    
    void Transfer(int);
    
    std::string model_array[11];
    ros::NodeHandle *n;
    
    ros::Subscriber GameState;
    long gamestate;

    //RobotNumber
    ros::Subscriber RobotNumber;
    ros::Subscriber TeamColor;
    long robotnumber;

    //gazebo_ModelStates subscriber
    ros::Subscriber Gazebo_Model_Name_sub;
    //ball subscriber
    ros::Subscriber ball_sub;
    
    //robot subscriber
    ros::Subscriber robot_1_pos_sub  ;
    ros::Subscriber robot_2_pos_sub  ;
    ros::Subscriber robot_3_pos_sub  ;
    ros::Subscriber robotOpt_1_pos_sub  ;
    ros::Subscriber robotOpt_2_pos_sub  ;
    ros::Subscriber robotOpt_3_pos_sub  ;

    ros::Subscriber Vision;
    ros::Subscriber Vision_Two_point;
    //robot publisher
    //no robot_1_role_pub, because robot_1 is always goal keeper
    ros::Publisher robot_1_role_pub;
    ros::Publisher robot_2_role_pub;
    ros::Publisher robot_3_role_pub;
    
    
    void find_gazebo_model_name_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){//----------------------------------printf here is ok, but printf next row will crash if i open over one robot map
        
        if(run_one) return;
        
        int model_length;
        model_length = msg->name.size();
        printf("model_length= %d",model_length);
        for(int i = 0; i<model_length;i++){
            model_array[i] = msg->name[i];
        }
        run_one = true;
    }
    int get_model_num(const std::string ModelName){
        int send_back_num;
        for(int i =0; i< 11; i++){
            if(model_array[i] == ModelName) return i;
        }
    }
    
    void subGameState(const std_msgs::Int32::ConstPtr &msg){
        global_env->gameState=msg->data;
    }

    void subTeamColor(const std_msgs::String::ConstPtr &msg){
        global_env->teamcolor= msg->data;
    }

    void  ball_sub_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("soccer");
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->currentBall.pos.x = tPose.position.x;
        global_env->currentBall.pos.y = tPose.position.y;
    }
    
    
    void  robot_1_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("R1");
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->home[0].pos.x = tPose.position.x;
        global_env->home[0].pos.y = tPose.position.y;
        Transfer(0);
        
        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->home[0].rotation = yaw;
    }
    
    void  robot_2_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("R2");
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->home[1].pos.x = tPose.position.x;
        global_env->home[1].pos.y = tPose.position.y;
        Transfer(1);
        
        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->home[1].rotation = yaw;
    }
    void  robot_3_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){

        int model_num;
        model_num = get_model_num("R3");
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->home[2].pos.x = tPose.position.x;
        global_env->home[2].pos.y = tPose.position.y;
        Transfer(2);
        
        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->home[2].rotation = yaw;
    }
    
    
    void  robotOpt_1_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
        global_env->opponent[0].pos.x = msg->pose.pose.position.x;
        global_env->opponent[0].pos.y = msg->pose.pose.position.y;
        
        double w = msg->pose.pose.orientation.w;
        double x = msg->pose.pose.orientation.x;
        double y = msg->pose.pose.orientation.y;
        double z = msg->pose.pose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->opponent[0].rotation = yaw;
    }
    
    
    void  robotOpt_2_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
        global_env->opponent[1].pos.x = msg->pose.pose.position.x;
        global_env->opponent[1].pos.y = msg->pose.pose.position.y;
        double w = msg->pose.pose.orientation.w;
        double x = msg->pose.pose.orientation.x;
        double y = msg->pose.pose.orientation.y;
        double z = msg->pose.pose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->opponent[1].rotation = yaw;
    }
    
    
    void  robotOpt_3_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
        global_env->opponent[2].pos.x = msg->pose.pose.position.x;
        global_env->opponent[2].pos.y = msg->pose.pose.position.y;
        double w = msg->pose.pose.orientation.w;
        double x = msg->pose.pose.orientation.x;
        double y = msg->pose.pose.orientation.y;
        double z = msg->pose.pose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->opponent[2].rotation = yaw;
    }

    void subVision(const vision::Object::ConstPtr &msg){
        double ball_distance,yellow_distance,blue_distance;
        yellow_distance = msg->yellow_dis;
        blue_distance = msg->blue_dis;
        if(global_env->teamcolor == "Blue"){
            global_env->home[global_env->RobotNumber].op_goal.distance= blue_distance/100;
            global_env->home[global_env->RobotNumber].op_goal.angle = msg->blue_ang;
            global_env->home[global_env->RobotNumber].goal.distance = yellow_distance/100;
            global_env->home[global_env->RobotNumber].goal.angle = msg->yellow_ang;

        }else if(global_env->teamcolor == "Yellow"){
            global_env->home[global_env->RobotNumber].op_goal.distance= yellow_distance/100;
            global_env->home[global_env->RobotNumber].op_goal.angle = msg->yellow_ang;
            global_env->home[global_env->RobotNumber].goal.distance= blue_distance/100;
            global_env->home[global_env->RobotNumber].goal.angle = msg->blue_ang;
        }

       ball_distance = msg->ball_dis;
       global_env->home[global_env->RobotNumber].ball.distance = ball_distance/100;
       global_env->home[global_env->RobotNumber].ball.angle = msg->ball_ang;
    }

    void subVision_Two_point(const vision::Two_point::ConstPtr &msg){
        if(global_env->teamcolor == "Blue"){
            int ang_max = msg->blue_ang_max;
            int ang_min = msg->blue_ang_min;

            global_env->home[global_env->RobotNumber].opgoal_edge.angle_max = ang_max;
            global_env->home[global_env->RobotNumber].opgoal_edge.angle_min = ang_min;
            double opgoal_left = msg->blue_left,opgoal_right = msg->blue_right;
            global_env->home[global_env->RobotNumber].opgoal_edge.left_dis = opgoal_left/100;
            global_env->home[global_env->RobotNumber].opgoal_edge.right_dis = opgoal_right/100;
        }else if(global_env->teamcolor == "Yellow"){
            int ang_max = msg->yellow_ang_max;
            int ang_min = msg->yellow_ang_min;
            global_env->home[global_env->RobotNumber].opgoal_edge.angle_max = ang_max;
            global_env->home[global_env->RobotNumber].opgoal_edge.angle_min = ang_min;
            double opgoal_left = msg->yellow_left,opgoal_right = msg->yellow_right;
            global_env->home[global_env->RobotNumber].opgoal_edge.left_dis = opgoal_left/100;
            global_env->home[global_env->RobotNumber].opgoal_edge.right_dis = opgoal_right/100;
        }
    }
    
};

#endif /* NODE_HPP_ */
