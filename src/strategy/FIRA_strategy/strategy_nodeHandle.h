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

#ifndef Strategy_nodeHandle_HPP_
#define Strategy_nodeHandle_HPP_




/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <string>
#include <iostream>
#include "../common/Env.h"
#include "../common/BaseNode.h"

//message inclue
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "vision/Object.h"
#include "FIRA_status_plugin/RobotSpeedMsg.h"
#include "FIRA_status_plugin/ModelMsg.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"

/*****************************************************************************
** Define
*****************************************************************************/
#define Ball_Topic_Name         "/FIRA/Strategy/WorldMap/soccer"
#define ModelState_Topic_Name  "/gazebo/model_states"
//robot prefix
#define Robot_Topic_Prefix "/FIRA/R"
#define RobotOpt_Topic_Prefix "/FIRA/Opt_R"
#define GameState_Topic "/FIRA/GameState"
#define TeamColor_Topic "/FIRA/TeamColor"
#define Vision_Topic "/vision/object"
//one_Robot speed
#define Robot_Topic_Speed "/cmd_vel"
//robot suffix
#define Robot_Position_Topic_Suffix "/Strategy/WorldMap/RobotPos"
#define Robot_Role_Topic_Suffix "/Strategy/Coach/role"
#define RobotSpeed_Topic_Suffix "/Strategy/PathPlan/RobotSpeed"

#define Node_Name "PersonalStrategy"

#define VectorMax 1.42
#define VectorMin 0.05
/*****************************************************************************
** Class
*****************************************************************************/

class Strategy_nodeHandle :public BaseNode {
public:
    Strategy_nodeHandle(int argc, char** argv);

    virtual ~Strategy_nodeHandle(){}

    void pubGrpSpeed();

    void setEnv(Environment *inEnv){
        global_env = inEnv;

    }

    int* getRoleAry(){
//        std::cout << "roleAry[0]=" <<   roleAry[0] << std::endl;
        return roleAry;
    }

    void setOpponent(bool inBool){opponent = inBool;}

    ros::NodeHandle* getNodeHandle(){return n;}
    long getGameState(){return gamestate;}
    std::string getTeamColor(){return teamcolor;}



protected:
    void ros_comms_init();

private:
    int roleAry[PLAYERS_PER_SIDE];
    std::string model_array[11];
    bool opponent;

    Environment *global_env;


    ros::NodeHandle *n;
    long gamestate;
    std::string teamcolor;

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

    ros::Subscriber GameState;
    ros::Subscriber TeamColor;
    ros::Subscriber Vision;

    //robot role publisher
    //no robot_1_role_sub, because robot_1 is always goal keeper
    ros::Subscriber robot_2_role_sub;
    ros::Subscriber robot_3_role_sub;

    //robot speed publisher
    ros::Publisher robot_1_speed_pub;
    ros::Publisher robot_2_speed_pub;
    ros::Publisher robot_3_speed_pub;
    ros::Publisher robotOpt_1_speed_pub;
    ros::Publisher robotOpt_2_speed_pub;
    ros::Publisher robotOpt_3_speed_pub;

    //one_robot speed
    ros::Publisher robot_speed_pub;

    bool run_one = false;

    void Transfer(int);

    double c_v_x,c_v_y;

    void rotateXY(double rotate,double inX,double inY,double &newX,double &newY);
    void pubSpeed(ros::Publisher *puber,double v_x,double v_y,double v_yaw,double robot_rot);
    void velocity_S_planning(geometry_msgs::Twist *msg);

    //find Gazebo_msgs::ModelStates name
    void find_gazebo_model_name_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){//----------------------------------printf here is ok, but printf next row will crash if i open over one robot map
//        printf("testttttttttttttt\n");

        if(run_one) return;

        int model_length;
        model_length = msg->name.size();
        /*
        for(int i =0;i<11;i++){
            if(msg->name[i].c_str()==NULL){
                model_length = i;
            }
            if(model_length!= 0) i = 11;
//            printf("%d\n",model_length);
        }*/
        printf("model_length= %d",model_length);
        for(int i = 0; i<model_length;i++){
            model_array[i] = msg->name[i];
//            printf("%d = %s\n",i,model_array[i].c_str());
        }
        run_one = true;
//        printf("%s\n",model_array[6].c_str());
    }
    int get_model_num(const std::string ModelName){
        int send_back_num;
        for(int i =0; i< 11; i++){
            if(model_array[i] == ModelName) return i;
        }
    }

//Use_topic_gazebo_msgs_Model_States to get model position
    void  ball_sub_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("soccer");
//        printf("model_num=%d\n",model_num);
//        ROS_INFO("name=%s",msg->name[1].c_str());
        geometry_msgs::Pose tPose = msg->pose[model_num];
//        ROS_INFO("x=%lf",tPose.position.x);
        global_env->currentBall.pos.x = tPose.position.x;
        global_env->currentBall.pos.y = tPose.position.y;

//        printf("ball_x = %lf,ball_y = %lf\n",  tPose.position.x, tPose.position.y);

    }
    void robot_1_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("R1");
//        printf("model_num=%d\n",model_num);
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
//        printf("yaw =%lf\n",yaw);

    }
    void robot_2_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("R2");
//        printf("model_num=%d\n",model_num);
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
//        printf("yaw =%lf\n",yaw);

    }
    void  robot_3_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("R3");
//        printf("model_num=%d\n",model_num);
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
//        printf("yaw =%lf\n",yaw);

    }


    void  robotOpt_1_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("FIRA_Opt_1");
//        printf("model_num=%d\n",model_num);
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->opponent[0].pos.x = tPose.position.x;
        global_env->opponent[0].pos.y = tPose.position.y;

        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->opponent[0].rotation = yaw;
//        printf("yaw =%lf\n",yaw);
    }


    void  robotOpt_2_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("FIRA_Opt_2");
//        printf("model_num=%d\n",model_num);
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->opponent[1].pos.x = tPose.position.x;
        global_env->opponent[1].pos.y = tPose.position.y;

        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->opponent[1].rotation = yaw;
//        printf("yaw =%lf\n",yaw);
    }


    void  robotOpt_3_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("FIRA_Opt_3");
//        printf("model_num=%d\n",model_num);
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->opponent[2].pos.x = tPose.position.x;
        global_env->opponent[2].pos.y = tPose.position.y;

        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->opponent[2].rotation = yaw;
//        printf("yaw =%lf\n",yaw);
    }
//    void  ball_sub_fun(const FIRA_status_plugin::ModelMsg::ConstPtr &msg){
//        global_env->currentBall.pos.x = msg->x;
//        global_env->currentBall.pos.y = msg->y;

//        //std::cout << "[ball] x=" << msg->x <<",y=" << msg->y << std::endl;
//    }

//    void  robot_1_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
//        global_env->home[0].pos.x = msg->pose.pose.position.x;
//        global_env->home[0].pos.y = msg->pose.pose.position.y;


//        double w = msg->pose.pose.orientation.w;
//        double x = msg->pose.pose.orientation.x;
//        double y = msg->pose.pose.orientation.y;
//        double z = msg->pose.pose.orientation.z;
//        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
//        global_env->home[0].rotation = yaw;

//        //std::cout << "[robot-1] x=" << global_env->home[0].pos.x <<",y=" << global_env->home[0].pos.y << std::endl;
//    }
//    void  robot_2_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
//        global_env->home[1].pos.x = msg->pose.pose.position.x;
//        global_env->home[1].pos.y = msg->pose.pose.position.y;

//        double w = msg->pose.pose.orientation.w;
//        double x = msg->pose.pose.orientation.x;
//        double y = msg->pose.pose.orientation.y;
//        double z = msg->pose.pose.orientation.z;
//        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
//        global_env->home[1].rotation = yaw;

//        //std::cout << "[robot-2] x=" << global_env->home[1].pos.x <<",y=" << global_env->home[1].pos.y << std::endl;

//    }
//    void  robot_3_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
//        global_env->home[2].pos.x = msg->pose.pose.position.x;
//        global_env->home[2].pos.y = msg->pose.pose.position.y;

//        double w = msg->pose.pose.orientation.w;
//        double x = msg->pose.pose.orientation.x;
//        double y = msg->pose.pose.orientation.y;
//        double z = msg->pose.pose.orientation.z;
//        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
//        global_env->home[2].rotation = yaw;

//        //std::cout << "[robot-3] x=" << global_env->home[2].pos.x <<",y=" << global_env->home[2].pos.y << std::endl;

//    }


//    void  robotOpt_1_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
//        global_env->opponent[0].pos.x = msg->pose.pose.position.x;
//        global_env->opponent[0].pos.y = msg->pose.pose.position.y;

//        double w = msg->pose.pose.orientation.w;
//        double x = msg->pose.pose.orientation.x;
//        double y = msg->pose.pose.orientation.y;
//        double z = msg->pose.pose.orientation.z;
//        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
//        global_env->opponent[0].rotation = yaw;

//        //std::cout << "[robotOpt-1] x=" << global_env->opponent[0].pos.x <<",y=" << global_env->opponent[0].pos.y << std::endl;

//    }


//    void  robotOpt_2_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
//        global_env->opponent[1].pos.x = msg->pose.pose.position.x;
//        global_env->opponent[1].pos.y = msg->pose.pose.position.y;
//        double w = msg->pose.pose.orientation.w;
//        double x = msg->pose.pose.orientation.x;
//        double y = msg->pose.pose.orientation.y;
//        double z = msg->pose.pose.orientation.z;
//        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
//        global_env->opponent[1].rotation = yaw;
//    }


//    void  robotOpt_3_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
//        global_env->opponent[2].pos.x = msg->pose.pose.position.x;
//        global_env->opponent[2].pos.y = msg->pose.pose.position.y;
//        double w = msg->pose.pose.orientation.w;
//        double x = msg->pose.pose.orientation.x;
//        double y = msg->pose.pose.orientation.y;
//        double z = msg->pose.pose.orientation.z;
//        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
//        global_env->opponent[2].rotation = yaw;
//    }


    void  robot_2_role_fun(const std_msgs::Int32::ConstPtr &msg){

        roleAry[1] = msg->data;
        //if(roleAry[1] == Role_Attack)   std::cout << "Robot-2 is Role_Attack"
         //std::cout << "Robot-2 is Role=" << roleAry[1] << std::endl;
    }

    void  robot_3_role_fun(const std_msgs::Int32::ConstPtr &msg){

        roleAry[2] = msg->data;

        //std::cout << "Robot-3 is Role=" << roleAry[2] << std::endl;
    }
    void subGameState(const std_msgs::Int32::ConstPtr &msg){
        gamestate=msg->data;
        //std::cout<<"receive"<<std::endl;
    }
    void subTeamColor(const std_msgs::String::ConstPtr &msg){
        teamcolor= msg->data;
        //std::cout<<"receive"<<std::endl;
    }
    void subVision(const vision::Object::ConstPtr &msg){
        global_env->home[0].ball.angle=msg->ball_ang;
        global_env->home[0].ball.distance=msg->ball_dis;
        global_env->home[0].op_goal.angle=msg->yellow_ang;
        global_env->home[0].op_goal.distance=msg->yellow_dis;
        global_env->home[0].goal.angle=msg->blue_ang;
        global_env->home[0].goal.distance=msg->blue_dis;

    }
};

#endif /* NODE_HPP_ */
