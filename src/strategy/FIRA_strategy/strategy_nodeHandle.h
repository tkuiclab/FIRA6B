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

//message include
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "vision/Object.h"
#include "vision/Two_point.h"
#include "FIRA_status_plugin/RobotSpeedMsg.h"
#include "FIRA_status_plugin/ModelMsg.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

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
#define Vision_Two_point_Topic "/interface/Two_point"
#define SAVEPARAM_TOPIC "/FIRA/SaveParam"
//BlackObject_distance
#define  BlackObject_Topic "/vision/BlackRealDis"
//one_Robot speed
#define Robot_Topic_Speed "/motion/cmd_vel"
//robot suffix
#define Robot_Position_Topic_Suffix "/Strategy/WorldMap/RobotPos"
#define Robot_Role_Topic_Suffix "/Strategy/Coach/role"
#define RobotSpeed_Topic_Suffix "/Strategy/PathPlan/RobotSpeed"

//BlackObject_distance
#define  BlackObject_Topic "/vision/BlackRealDis"

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
    void setOpponent(bool inBool){opponent = inBool;}
    // shoot signal
    ros::Publisher shoot;
    // pub shoot signal
    void pubShoot(int shoot_value){
        ros::Time current = ros::Time::now();
        static double current_time =10000;
        static double record_time;
        double time = current_time-record_time;
        if(time>1){
            std_msgs::Int32 shoot_signal;
            shoot_signal.data = shoot_value;
            shoot.publish(shoot_signal);
            record_time = current_time;
        }
        current_time = (double)(current.sec+(double)current.nsec/1000000000);

    }
    struct SaveAry{
            int distance;
            int location;
            int counter;
        };
    struct SaveAry Save[20];
    struct New_SaveAry{
            int distance;
            int location;
            int counter;
            int middle;
            int middle_place;
        };
    struct New_SaveAry New_Save[20];

    int* getRoleAry(){
        return roleAry;
    }

    ros::NodeHandle* getNodeHandle(){return n;}
    long getGameState(){return gamestate;}
    std::string getTeamColor(){return teamcolor;}
    int getIsSimulator(){return IsSimulator;}

    //BlackObject
    int Blackangle;
    int *blackobject;
    void loadParam(ros::NodeHandle *n);
    int* getBlackObject(){return blackobject;}
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
    ros::Subscriber Vision_Two_point;

    //BlackObject
    ros::Subscriber BlackObject;

    //robot role publisher
    //no robot_1_role_sub, because robot_1 is always goal keeper
    ros::Subscriber robot_2_role_sub;
    ros::Subscriber robot_3_role_sub;
    ros::Subscriber SAVEPARAM;
    std::vector<double> Chase_Strategy;
    ros::Subscriber another_robot_info_sub;

    //robot speed publisher
    ros::Publisher robot_1_speed_pub;
    ros::Publisher robot_2_speed_pub;
    ros::Publisher robot_3_speed_pub;
    ros::Publisher robotOpt_1_speed_pub;
    ros::Publisher robotOpt_2_speed_pub;
    ros::Publisher robotOpt_3_speed_pub;

    //one_robot speed
    ros::Publisher robot_speed_pub;

    /// load param begin
    std::vector<double> SPlanning_Velocity;
    std::vector<double> Distance_Settings;
    std::vector<int> Scan;
    int IsSimulator;
    /// load param end

    bool run_one = false;

    void Transfer(int);

    double c_v_x,c_v_y;

    void rotateXY(double rotate,double inX,double inY,double &newX,double &newY);
    void pubSpeed(ros::Publisher *puber,double v_x,double v_y,double v_yaw,double robot_rot);
    void velocity_S_planning(geometry_msgs::Twist *msg);

    //find Gazebo_msgs::ModelStates name
    void find_gazebo_model_name_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){//----------------------------------printf here is ok, but printf next row will crash if i open over one robot map
        if(run_one) return;
        int model_length;
        model_length = msg->name.size();
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

//Use_topic_gazebo_msgs_Model_States to get model position
    void  ball_sub_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("soccer");
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->currentBall.pos.x = tPose.position.x;
        global_env->currentBall.pos.y = tPose.position.y;
    }
    void robot_1_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
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
    void robot_2_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
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


    void  robotOpt_1_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("FIRA_Opt_1");
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->opponent[0].pos.x = tPose.position.x;
        global_env->opponent[0].pos.y = tPose.position.y;

        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->opponent[0].rotation = yaw;
    }


    void  robotOpt_2_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("FIRA_Opt_2");
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->opponent[1].pos.x = tPose.position.x;
        global_env->opponent[1].pos.y = tPose.position.y;

        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->opponent[1].rotation = yaw;
    }


    void  robotOpt_3_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("FIRA_Opt_3");
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->opponent[2].pos.x = tPose.position.x;
        global_env->opponent[2].pos.y = tPose.position.y;

        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->opponent[2].rotation = yaw;
    }

    void  robot_2_role_fun(const std_msgs::Int32::ConstPtr &msg){
        roleAry[1] = msg->data;
    }

    void  robot_3_role_fun(const std_msgs::Int32::ConstPtr &msg){
        roleAry[2] = msg->data;
    }
    void subGameState(const std_msgs::Int32::ConstPtr &msg){
        gamestate=msg->data;
    }
    void subTeamColor(const std_msgs::String::ConstPtr &msg){
        teamcolor= msg->data;
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
            int ang1 = msg->yellow_ang1;
            int ang2 = msg->yellow_ang2;

            if(ang1>180){
                ang1=ang1-360;
            }
            ang1=ang1+90;
            if(ang1>180){
                ang1=ang1-360;
            }else if(ang1<-180){
                ang1=ang1+360;
            }

            if(ang2>180){
                ang2=ang2-360;
            }
            ang2=ang2+90;
            if(ang2>180){
                ang2=ang2-360;
            }else if(ang2<-180){
                ang2=ang2+360;
            }

            global_env->home[global_env->RobotNumber].goal_edge.angle_1 = ang1;
            global_env->home[global_env->RobotNumber].goal_edge.angle_2 = ang2;

        }else if(global_env->teamcolor == "Yellow" && msg->yellow_ang1 != msg->yellow_ang2){
            double ang1 = msg->blue_ang1;
            double ang2 = msg->blue_ang2;

            if(ang1>180){
                ang1=ang1-360;
            }
            ang1=ang1+90;
            if(ang1>180){
                ang1=ang1-360;
            }else if(ang1<-180){
                ang1=ang1+360;
            }

            if(ang2>180){
                ang2=ang2-360;
            }
            ang2=ang2+90;
            if(ang2>180){
                ang2=ang2-360;
            }else if(ang2<-180){
                ang2=ang2+360;
            }

            global_env->home[global_env->RobotNumber].goal_edge.angle_1 = ang1;
            global_env->home[global_env->RobotNumber].goal_edge.angle_2 = ang2;
        }
    }
    void subBlackObject(const std_msgs::Int32MultiArray::ConstPtr &msg){
        static int counter=0;
        int All_Line_distance[360];
        int angle[360];
        int place;
        Range Unscan[3];
        Unscan[0].begin = Scan[0] - (Scan[3]-1);
        Unscan[0].end   = Scan[0] + (Scan[3]-1);
        Unscan[1].begin = Scan[1] - (Scan[4]-1);
        Unscan[1].end   = Scan[1] + (Scan[4]-1);
        Unscan[2].begin = Scan[2] - (Scan[4]-1);
        Unscan[2].end   = Scan[2] + (Scan[4]-1);
        for(int i=0;i<360/Blackangle;i++){
//            if((i*Blackangle >= Unscan[0].begin && i*Blackangle <= Unscan[0].end )||\
//            (i*Blackangle >= Unscan[1].begin && i*Blackangle >= Unscan[1].end) ||\
//            (i*Blackangle >= Unscan[2].begin && i*Blackangle >= Unscan[2].end))
//                All_Line_distance[i] = -1;
//            else{
//            printf("0.begin=%d\t0.end=%d\n",Unscan[0].begin,Unscan[0].end);
//            printf("1.begin=%d\t1.end=%d\n",Unscan[1].begin,Unscan[1].end);
//            printf("2.begin=%d\t2.end=%d\n",Unscan[2].begin,Unscan[2].end);
//              printf("All_Line_distance[%d]=%d\n",All_Line_distance[i],i);
                All_Line_distance[i] = msg -> data[i];
//        }
        }

        global_env->mindis[0] = All_Line_distance[0];
        for(int i=1;i<360/Blackangle;i++){
//            printf("%d\t%d\n",i,All_Line_distance[i]);
            if(All_Line_distance[i] < global_env->mindis[0]){
                if(All_Line_distance[i]>25 &&All_Line_distance[i]<1000){						
                    global_env->mindis[0] = All_Line_distance[i];
                    place = i*Blackangle;
                }
            }
        }
        global_env->blackangle[0] = place*Blackangle;
        if(global_env->blackangle[0] > 180){
            global_env->blackangle[0] = -(360 - global_env->blackangle[0]);
        }
        //        //---------------------------vincent--1 meter all around------------------
        //        double distance_br = global_env->home[global_env->RobotNumber].ball.distance;
        //        double distance_dr = global_env->home[global_env->RobotNumber].goal.distance;
        //        double op_distance_dr = global_env->home[global_env->RobotNumber].op_goal.distance;
        //        double angle_br = global_env->home[global_env->RobotNumber].ball.angle;
        //        double angle_dr = global_env->home[global_env->RobotNumber].goal.angle;
        //        double op_angle_dr = global_env->home[global_env->RobotNumber].op_goal.angle;
        //        double transform_angle_br=angle_br;
        //        //let ball angle = black line angle
        //        int ignore_limit_angle=30;
        //        int ignore_upper=0;
        //        int ignore_lower=0;
        //        int ignore_360upper=0;
        //        int ignore_360lower=0;
        //        int ignore_720upper=0;
        //        int ignore_720lower=0;
        //        int limit_obstacle_dis=100;
        //        int limit_obstacle_dis_counter=0;
        //        int first_obstacle_clear=0;
        //        int define_obstacle_angle=15;
        //        double obstacle_middle_angle=0;
        //        int ignore_counter=0;
        //        double final_angle=0;
        //        double final_distance=0;
        //        int end_search = 0;
        //        int i_limit=0;
        //        int obstacle_distance=0;
        //        int last_obstacle_distance=0;
        //        if(transform_angle_br<0){//let ball_angle be 0~360 degree
        //            transform_angle_br = transform_angle_br + 360;
        //        }
        //        if(transform_angle_br < ignore_limit_angle){// prepare to scan two round
        //            transform_angle_br = transform_angle_br + 360;
        //        }
        //        if(transform_angle_br<360){
        //            ignore_360upper=transform_angle_br+360+ignore_limit_angle;
        //            ignore_360lower=transform_angle_br+360-ignore_limit_angle;
        //            ignore_upper=transform_angle_br+30;
        //            ignore_lower=transform_angle_br-30;
        //        }else{
        //            ignore_360upper=transform_angle_br+30;
        //            ignore_360lower=transform_angle_br-30;
        //            ignore_upper=transform_angle_br-360;
        //            ignore_lower=0;
        //        }
        //        if(ignore_360upper>720){
        //            ignore_720upper = ignore_360upper-720;
        //            ignore_720lower = 0;
        //        }

        //        for(i=0; i<720/Blackangle; i++){
        //            if(end_search==1){// end_search jump out for loop
        //                break;
        //            }

        //            if(i*Blackangle>360||(i==360/Blackangle)){// if searching round two
        //                i_limit=360/Blackangle;
        //            }else{
        //                i_limit=0;
        //            }
        //            if((All_Line_distance[0]<limit_obstacle_dis)&&(i==0)){//something at 0 degree
        //                first_obstacle_clear=1;// first obstacle will clear
        //            }else if(i*Blackangle>=180&&first_obstacle_clear==1){// if across 180 degree still haven't find first obstacle
        //                first_obstacle_clear=0;
        //            }
        //            if(All_Line_distance[i-i_limit]<limit_obstacle_dis){//something within distance
        //                limit_obstacle_dis_counter++;
        //                obstacle_distance = obstacle_distance + All_Line_distance[i-i_limit];
        //                obstacle_middle_angle = i*Blackangle-limit_obstacle_dis_counter*Blackangle/2;
        //                last_obstacle_distance = All_Line_distance[i-i_limit];
        //                printf("limit_obstacle_dis_counter=%d,i=%d,i_limit=%d,angle[%d]=%d\n",limit_obstacle_dis_counter,i,i_limit,i*Blackangle,All_Line_distance[i-i_limit]);
        //                ignore_counter=0;// something there, so get ignore chance
        //            }else{// not within distance
        //                ignore_counter++;//give chance to ignore
        //                if(ignore_counter>=2){//really nothing there
        //                    if(limit_obstacle_dis_counter*Blackangle>define_obstacle_angle){// if last obstacle large enough
        //                        if(first_obstacle_clear==1){//first time dont restore
        //                            first_obstacle_clear--;
        //                            printf("first_obstacle_clear\n");
        //                        }else{
        //                            final_angle = obstacle_middle_angle; // restore
        //                            final_distance = obstacle_distance/limit_obstacle_dis_counter;
        //                            end_search = 1;// get final angle, so end search
        //                            printf("end search\n");
        //                        }
        //                    }
        //                    if(limit_obstacle_dis_counter>=2){// if something there but not large enough, still clear first time counter;
        //                        first_obstacle_clear--;
        //                    }
        //                    printf("ignore_counter>2,clear all\n");
        //                    limit_obstacle_dis_counter = 0;// reset obstacle
        //                    obstacle_distance = 0;
        //                    last_obstacle_distance = 0;
        //                }else{//keep counting
        //                    limit_obstacle_dis_counter++;
        //                    obstacle_distance = obstacle_distance + last_obstacle_distance;
        //                    obstacle_middle_angle = i*Blackangle-limit_obstacle_dis_counter*Blackangle/2;
        //                    printf("limit_obstacle_dis_counter=%d,i=%d,i_limit=%d,angle[%d]=%d\n",limit_obstacle_dis_counter,i,i_limit,i*Blackangle,All_Line_distance[i-i_limit]);

        //                }
        //            }
        //            if(((i*Blackangle>=ignore_lower)&&(i*Blackangle<=ignore_upper)||(i*Blackangle>=ignore_360lower)&&(i*Blackangle<=ignore_360upper))||(i*Blackangle>=ignore_720lower)&&(i*Blackangle<=ignore_720upper)){
        //                // if angle within these area, will clear
        //                limit_obstacle_dis_counter=0;
        //                ignore_counter=0;
        //                printf("angle within these area, will clear\n");
        //            }
        //        }
        //        if(final_angle>540&&final_angle<=720){
        //            final_angle=final_angle-720;
        //        }else if(final_angle>180&&final_angle<=540){
        //            final_angle=final_angle-360;
        //        }
        //        global_env->Support_Obstacle_angle=final_angle;
        //        global_env->Support_Obstacle_distance= final_distance/100;
        //        printf("final angle = %f \n",final_angle);
        //        printf("final_distance = %f \n",final_distance/100);
        //        printf("ball_distance = %f\n",distance_br);
        //        printf("transform_angle_br=%f\n",transform_angle_br);
        //        printf("ignore_upper=%d\n",ignore_upper);
        //        printf("ignore_lower=%d\n",ignore_lower);
        //        printf("ignore_360upper=%d\n",ignore_360upper);
        //        printf("ignore_360lower=%d\n",ignore_360lower);
        //        printf("ignore_720upper=%d\n",ignore_720upper);
        //        printf("ignore_720lower=%d\n",ignore_720lower);


        double distance_br = global_env->home[global_env->RobotNumber].ball.distance;
        double distance_dr = global_env->home[global_env->RobotNumber].goal.distance;
        double op_distance_dr = global_env->home[global_env->RobotNumber].op_goal.distance;
        double angle_br = global_env->home[global_env->RobotNumber].ball.angle;
        double angle_dr = global_env->home[global_env->RobotNumber].goal.angle;
        double op_angle_dr = global_env->home[global_env->RobotNumber].op_goal.angle;
        double transform_angle_br=angle_br;
        //let ball angle = black line angle
        int i=0;
        int ignore_limit_angle=25;
        int ignore_upper=0;
        int ignore_lower=0;
        int ignore_360upper=0;
        int ignore_360lower=0;
        int ignore_720upper=0;
        int ignore_720lower=0;
        int limit_obstacle_dis=200;
        int limit_obstacle_dis_counter=0;
        int first_obstacle_clear=0;
        int define_obstacle_angle=12;
        double obstacle_middle_angle=0;
        int ignore_counter=0;
        double final_angle=999;
        double final_distance=999;
        int end_search = 0;
        int i_limit=0;
        int obstacle_distance=0;
        int last_obstacle_distance=0;
        int degree_controller = 0; // if ball dis lower, start scan 180 degree
        int edge_controller = 1;
        static int edge_count=0;
        static int Begin_time = 0;
        static int Current_time = 0;
        Current_time = ros::Time::now().toSec();
        if(global_env->gameState==0){
            Begin_time = ros::Time::now().toSec();
            Current_time = ros::Time::now().toSec();
        }
        if((abs(Current_time-Begin_time)<2)&&(global_env->gameState==5)){
           degree_controller=270;
           ignore_limit_angle=25;
           limit_obstacle_dis=200;
           edge_controller = 1;
        }else{
           degree_controller=0;
           ignore_limit_angle=45;
           limit_obstacle_dis=200;
           edge_controller = 0;
        }



        if(transform_angle_br<0){//let ball_angle be 0~360 degree
            transform_angle_br = transform_angle_br + 360;
        }
        if(transform_angle_br < ignore_limit_angle){// prepare to scan two round
            transform_angle_br = transform_angle_br + 360;
        }
        if(transform_angle_br<360){
            ignore_360upper=transform_angle_br+360+ignore_limit_angle;
            ignore_360lower=transform_angle_br+360-ignore_limit_angle;
            ignore_upper=transform_angle_br+ignore_limit_angle;
            ignore_lower=transform_angle_br-ignore_limit_angle;
        }else{
            ignore_360upper=transform_angle_br+ignore_limit_angle;
            ignore_360lower=transform_angle_br-ignore_limit_angle;
            ignore_upper=transform_angle_br-360+ignore_limit_angle;
            ignore_lower=0;
        }
        if(ignore_360upper>720){
            ignore_720upper = ignore_360upper-720;
            ignore_720lower = 0;
        }
        for(i=degree_controller/Blackangle; i<720/Blackangle; i++){
            if(end_search==1){// end_search jump out for loop
                break;
            }

            if(i*Blackangle>360||(i==360/Blackangle)){// if searching round two
                i_limit=360/Blackangle;
            }else{
                i_limit=0;
            }
            if((All_Line_distance[0]<limit_obstacle_dis)&&(i==0)){//something at 0 degree
                first_obstacle_clear=1;// first obstacle will clear
            }else if(i*Blackangle>=180&&first_obstacle_clear==1){// if across 180 degree still haven't find first obstacle
                first_obstacle_clear=0;
            }
            if(All_Line_distance[i-i_limit]<limit_obstacle_dis){//something within distance
                limit_obstacle_dis_counter++;
                obstacle_distance = obstacle_distance + All_Line_distance[i-i_limit];
                obstacle_middle_angle = i*Blackangle-limit_obstacle_dis_counter*Blackangle/2;
                last_obstacle_distance = All_Line_distance[i-i_limit];
                //printf("limit_obstacle_dis_counter=%d,i=%d,i_limit=%d,angle[%d]=%d\n",limit_obstacle_dis_counter,i,i_limit,i*Blackangle,All_Line_distance[i-i_limit]);
               // printf("obstacle_distance=%d\n",obstacle_distance);
                ignore_counter=0;// something there, so get ignore chance
            }else{// not within distance
                ignore_counter++;//give chance to ignore
                if(ignore_counter>=2){//really nothing there
                    if(limit_obstacle_dis_counter*Blackangle>define_obstacle_angle){// if last obstacle large enough
                        if(first_obstacle_clear==1){//first time dont restore
                            first_obstacle_clear--;
                            //printf("first_obstacle_clear\n");
                        }else{//decide restore or abandon information
                            if(obstacle_distance/limit_obstacle_dis_counter<final_distance){// restore all information
                                final_distance = obstacle_distance/limit_obstacle_dis_counter;
                                final_angle = obstacle_middle_angle;
                            }
//                            final_angle = obstacle_middle_angle; // restore
//                            //printf("obstacle_distance=%d\n",obstacle_distance);
//                            final_distance = obstacle_distance/limit_obstacle_dis_counter;
//                            end_search = 1;// get final angle, so end search
                            limit_obstacle_dis_counter=0;
                            ignore_counter=0;
                            obstacle_distance=0;
                            //printf("end search\n");
                        }
                    }
                    if(limit_obstacle_dis_counter>=2){// if something there but not large enough, still clear first time counter;
                        first_obstacle_clear--;
                    }
                    //printf("ignore_counter>2,clear all\n");
                    limit_obstacle_dis_counter = 0;// reset obstacle
                    obstacle_distance = 0;
                    last_obstacle_distance = 0;
                }else{//keep counting
                    limit_obstacle_dis_counter++;
                    obstacle_distance = obstacle_distance + last_obstacle_distance;
                    obstacle_middle_angle = i*Blackangle-limit_obstacle_dis_counter*Blackangle/2;
                    //printf("obstacle_distance=%d\n",obstacle_distance);
                    //printf("limit_obstacle_dis_counter=%d,i=%d,i_limit=%d,angle[%d]=%d\n",limit_obstacle_dis_counter,i,i_limit,i*Blackangle,All_Line_distance[i-i_limit]);

                }

            }
            if(((i*Blackangle>=ignore_lower)&&(i*Blackangle<=ignore_upper)||(i*Blackangle>=ignore_360lower)&&(i*Blackangle<=ignore_360upper))||(i*Blackangle>=ignore_720lower)&&(i*Blackangle<=ignore_720upper)){
                // if angle within these area, will clear
                if(limit_obstacle_dis_counter*Blackangle>define_obstacle_angle){
                    if(obstacle_distance/limit_obstacle_dis_counter<final_distance){// restore all information
                        final_distance = obstacle_distance/limit_obstacle_dis_counter;
                        final_angle = obstacle_middle_angle;
                    }
//                    final_angle = obstacle_middle_angle; // restore
//                    //printf("obstacle_distance=%d\n",obstacle_distance);
//                    final_distance = obstacle_distance/limit_obstacle_dis_counter;
//                    end_search = 1;// get final angle, so end search
                    //printf("end search\n");
                }
                limit_obstacle_dis_counter=0;
                ignore_counter=0;
                obstacle_distance=0;
                //printf("limit_obstacle_dis_counter=%d,i=%d,i_limit=%d,angle[%d]=%d\n",limit_obstacle_dis_counter,i,i_limit,i*Blackangle,All_Line_distance[i-i_limit]);
                //printf("angle within these area, will clear\n");
            }else if(((i*Blackangle>=450)&&(i*Blackangle<=630))&& (edge_controller==1)){
                if(limit_obstacle_dis_counter*Blackangle>define_obstacle_angle){
                    if(obstacle_distance/limit_obstacle_dis_counter<final_distance){// restore all information
                        final_distance = obstacle_distance/limit_obstacle_dis_counter;
                        final_angle = obstacle_middle_angle;
                    }
//                    final_angle = obstacle_middle_angle; // restore
//                    //printf("obstacle_distance=%d\n",obstacle_distance);
//                    final_distance = obstacle_distance/limit_obstacle_dis_counter;
//                    end_search = 1;// get final angle, so end search
//                    printf("end search\n");
                }
                limit_obstacle_dis_counter=0;
                ignore_counter=0;
                obstacle_distance=0;
                //printf("limit_obstacle_dis_counter=%d,i=%d,i_limit=%d,angle[%d]=%d\n",limit_obstacle_dis_counter,i,i_limit,i*Blackangle,All_Line_distance[i-i_limit]);
                //printf("infornt of woods, will clear\n");
            }
        }
        if(final_angle>540&&final_angle<=720){
            final_angle=final_angle-720;
        }else if(final_angle>180&&final_angle<=540){
            final_angle=final_angle-360;
        }
        global_env->Support_Obstacle_angle=final_angle;
        global_env->Support_Obstacle_distance= final_distance/100;
//        printf("final angle = %f \n",final_angle);
//        printf("ball angle=%f\n",angle_br);
//        printf("final_distance = %f \n",final_distance/100);
//        printf("ball_distance = %f\n",distance_br);
//        printf("transform_angle_br=%f\n",transform_angle_br);
//        printf("ignore_upper=%d\n",ignore_upper);
//        printf("ignore_lower=%d\n",ignore_lower);
//        printf("ignore_360upper=%d\n",ignore_360upper);
//        printf("ignore_360lower=%d\n",ignore_360lower);
//        printf("ignore_720upper=%d\n",ignore_720upper);
//        printf("ignore_720lower=%d\n",ignore_720lower);

  }
    void  another_robot_info(const std_msgs::Float32MultiArray::ConstPtr &msg){
        global_env->AnotherRobotNumber=msg->data[0];//another robot number
        global_env->AnotherBallDistance=msg->data[1];//another robot Ball distance
        global_env->home[global_env->AnotherRobotNumber].ball.distance=msg->data[1];
        global_env->AnotherGetBall=msg->data[2];//another robot is get ball (Yes=1,No=0)
        global_env->AnotherGoalDistance=msg->data[3];//another robot Goal distance
        global_env->home[global_env->AnotherRobotNumber].goal.distance=msg->data[3];
        global_env->R1OrderR2=msg->data[4];

    }
     void getSaveParam(const std_msgs::Int32::ConstPtr &msg){
         global_env->SaveParam = msg->data;
     }

};

#endif /* NODE_HPP_ */
