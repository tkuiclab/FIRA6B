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
#include "std_msgs/Float32MultiArray.h"
#include "FIRA_status_plugin/RobotSpeedMsg.h"
#include "FIRA_status_plugin/ModelMsg.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"
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

#include "vision/Object.h"
#include "std_msgs/Int32MultiArray.h"
#define Vision_Topic "/vision/object"
#define  BlackObject_Topic "/vision/BlackRealDis"
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
        std_msgs::Int32 robot_2_role;
        robot_2_role.data = roleAry[1]; //roleAry[2-1]
        robot_2_role_pub.publish(robot_2_role);
        
        
        std_msgs::Int32 robot_3_role;
        robot_3_role.data = roleAry[2];  //roleAry[3-1]
        robot_3_role_pub.publish(robot_3_role);
    }
    void pubOrder(int *msg){
        sendOrder = *msg;
    }
    void this_robot_info_publish(int rolearray){
        std_msgs::Float32MultiArray pubMsg;
        pubMsg.data.push_back(rolearray);
        pubMsg.data.push_back(global_env->home[rolearray].ball.distance);
        //Chase_Strategy[3]=angle, [4]=dis
        if((fabs(Chase_Strategy[3])>=fabs(global_env->home[rolearray].ball.angle))&&(Chase_Strategy[4]>=global_env->home[rolearray].ball.distance)){
          pubMsg.data.push_back(1);
        }else{
          pubMsg.data.push_back(0);
        }
        pubMsg.data.push_back(global_env->home[rolearray].goal.distance);

        pubMsg.data.push_back(sendOrder);
//        printf("Chase_Strategy[3]=%lf\n",fabs(Chase_Strategy[3]));
//        printf("ball.angle=%lf\n",global_env->home[rolearray].ball.angle);
//        printf("Chase_Strategy[4]=%lf\n",fabs(Chase_Strategy[4]));
//        printf("ball.distance=%lf\n",global_env->home[rolearray].ball.distance);
        this_robot_info_pub.publish(pubMsg);

    }
    ros::Publisher this_robot_info_pub;
    int Blackangle;
    int *blackobject;
    void loadParam(ros::NodeHandle *n);
    int* getBlackObject(){return blackobject;}
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

    int sendOrder;

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
    ros::Subscriber BlackObject;
    
    //robot publisher
    //no robot_1_role_pub, because robot_1 is always goal keeper
    ros::Publisher robot_2_role_pub;
    ros::Publisher robot_3_role_pub;
    
    ros::Subscriber another_robot_info_sub;
    /// load param begin
    std::vector<double> SPlanning_Velocity;
    std::vector<double> Distance_Settings;
    std::vector<double> Chase_Strategy;
    /// load param end
    void  another_robot_info(const std_msgs::Float32MultiArray::ConstPtr &msg){

        global_env->AnotherRobotNumber=msg->data[0];//another robot number
        global_env->AnotherBallDistance=msg->data[1];//another robot Ball distance
        global_env->home[global_env->AnotherRobotNumber].ball.distance=msg->data[1];
        global_env->AnotherGetBall=msg->data[2];//another robot is get ball (Yes=1,No=0)
        global_env->AnotherGoalDistance=msg->data[3];//another robot Goal distance
        global_env->home[global_env->AnotherRobotNumber].goal.distance=msg->data[3];
        global_env->R1OrderR2=msg->data[4];
//        printf("msg->data[0]=%f\n",msg->data[0]);
//        printf("msg->data[1]=%f\n",msg->data[1]);
//        printf("msg->data[2]=%f\n",msg->data[2]);
//        printf("msg->data[3]=%f\n",msg->data[3]);
//        printf("msg->data[3]=%f\n",msg->data[4]);
    }

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
    void subBlackObject(const std_msgs::Int32MultiArray::ConstPtr &msg){
        static int counter=0;
        int All_Line_distance[360];
        int angle[360];
        int place;

        for(int i=0; i<360/Blackangle-23; i++){
            All_Line_distance[i] = msg -> data[i];
        }
        global_env->mindis[0] = All_Line_distance[0];
        for(int i=1; i<360/Blackangle-23; i++){
            if(All_Line_distance[i] < global_env->mindis[0]){
                if(All_Line_distance[i]>25 &&All_Line_distance[i]<1000){
                    global_env->mindis[0] = All_Line_distance[i];
                    place = i;
                }
            }
        }

     global_env->blackangle[0] = place*3;
     if(global_env->blackangle[0] > 180){
        global_env->blackangle[0] = -(360 - global_env->blackangle[0]);
     }
    }
};

#endif /* NODE_HPP_ */
