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
#define IsTeamStrategy_Topic "/FIRA/IsTeamStrategy"
//BlackObject_distance
#define  BlackObject_Topic "/vision/BlackRealDis"
//TwoPointDoor
#define  Two_point_Topic "interface/Two_point"
//one_Robot speed
#define Robot_Topic_Speed "/motion/cmd_vel"
//robot suffix
#define Robot_Position_Topic_Suffix "/Strategy/WorldMap/RobotPos"
#define Robot_Role_Topic_Suffix "/Strategy/Coach/role"
#define RobotSpeed_Topic_Suffix "/Strategy/PathPlan/RobotSpeed"

//BlackObject_distance
#define  BlackObject_Topic "/vision/BlackRealDis"
#define  WhiteObject_Topic "/vision/WhiteRealDis"

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
    //apf_test

//    std::vector<int>angle_start;
//    std::vector<int>apf_dis;
//    std::vector<int>angle_end;
    //apf_test_ptr
    struct node{
        int angle_start;
        int angle_end;
        int distance;
        struct node *link;
    };
    typedef struct node NodeStruct;
    typedef NodeStruct *NodePtr;
    NodePtr head=NULL;
    int number_obstacle=0;

    NodePtr creat_node(int ang_start,int ang_end,int dis){
        NodePtr newnode = (NodePtr)malloc(sizeof(node));
        newnode->angle_start = ang_start;
        newnode->angle_end = ang_end;
        newnode->distance = dis;
        newnode->link = NULL;
        return newnode;
    }

    NodePtr append_node(NodePtr p)
    {
        if(head==NULL)
        {
            head=p;
            p->link=NULL;
        }
        else
        {
            p->link=head;
            head=p;
        }
        return head;
    }

    void display_list(NodePtr p){
        p=head;

        while (p!=NULL)
        {
            printf("ang_start:%d\tang_end:%d\t dis_avg:%d\n",p->angle_start,p->angle_end,p->distance);
            p=p->link;
        }
        printf("\n");
    }
    void list_to_global(NodePtr p){
        p=head;
        while (p!=NULL)
        {
            global_env->global_angle_end.push_back(p->angle_end);
            global_env->global_angle_start.push_back(p->angle_start);
            global_env->global_apf_dis.push_back(p->distance);
            p=p->link;
        }
        //printf("\n");
    }
    void delete_test(NodePtr p){
        p=head;

        while (p!=NULL)
        {
            if(p->distance<7){
                head=delete_node(p);
            }
            p=p->link;

        }
        printf("\n");
    }

    void list_filter(NodePtr p){
        p=head;
        while (p!=NULL)
        {
            if(p->link==NULL){
                break;
            }
            else{
                if((p->angle_start-(p->link->angle_end))<=3*Blackangle+1&&abs(p->distance-(p->link->distance))<=20)
                {
                    int line1=(p->angle_end-p->angle_start)/Blackangle;
                    int line2=(p->link->angle_end-p->link->angle_start)/Blackangle;
                    p->angle_start=p->link->angle_start;
                    p->distance=(p->distance*line1+p->link->distance*line2)/(line1+line2);
                    head=delete_node(p->link);
                    number_obstacle--;
                }
            }
            p=p->link;
        }
    }

    NodePtr delete_node(NodePtr p){
        NodePtr prev_node;
        if(p==head){
            head =head->link;
            return  head;
        }
        else
        {
            prev_node = head;
            while(prev_node->link!=p)
            {
                prev_node = prev_node->link;
            }
            if (p->link == NULL) {
                prev_node->link =NULL;
            }
            else
            {
                prev_node->link=p->link;
            }

        }
        free(p);
        return head;
    }

    void freelist(void)
    {
        NodePtr next_node;

        while(head!=NULL)
        {
            next_node=head->link;
            free(head);
            head=next_node;
        }
        head=NULL;

    }
    //BlackObject
    int Blackangle;
    int Whiteangle;
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
    ros::Subscriber IsTeamStrategy;

    //BlackObject
    ros::Subscriber BlackObject;
    ros::Subscriber WhiteObject;

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
    std::vector<int> Strategy_Selection;
    std::vector<int> Support_Strategy;
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
    void subIsTeamStrategy(const std_msgs::Int32::ConstPtr &msg){
        global_env->isteamstrategy=msg->data;
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

            global_env->home[global_env->RobotNumber].goal_large_area.angle = msg->yellow_fix_ang;
            global_env->home[global_env->RobotNumber].goal_large_area.distance = msg->yellow_fix_dis;
            global_env->home[global_env->RobotNumber].op_goal_large_area.angle = msg->blue_fix_ang;
            global_env->home[global_env->RobotNumber].op_goal_large_area.distance = msg->blue_fix_dis;

        }else if(global_env->teamcolor == "Yellow"){
            global_env->home[global_env->RobotNumber].op_goal.distance= yellow_distance/100;
            global_env->home[global_env->RobotNumber].op_goal.angle = msg->yellow_ang;
            global_env->home[global_env->RobotNumber].goal.distance= blue_distance/100;
            global_env->home[global_env->RobotNumber].goal.angle = msg->blue_ang;

            global_env->home[global_env->RobotNumber].goal_large_area.angle = msg->blue_fix_ang;
            global_env->home[global_env->RobotNumber].goal_large_area.distance = msg->blue_fix_dis;
            global_env->home[global_env->RobotNumber].op_goal_large_area.angle = msg->yellow_fix_ang;
            global_env->home[global_env->RobotNumber].op_goal_large_area.distance = msg->yellow_fix_dis;
        }

       ball_distance = msg->ball_dis;
       global_env->home[global_env->RobotNumber].ball.distance = ball_distance/100;
       global_env->home[global_env->RobotNumber].ball.angle = msg->ball_ang;

    }
    void subVision_Two_point(const vision::Two_point::ConstPtr &msg){
        global_env->blue_side_goal_data[0]=msg->blue_dis;
        global_env->yellow_side_goal_data[0]=msg->yellow_dis;
        global_env->blue_side_goal_data[1]=msg->blue_ang_max;
        global_env->blue_side_goal_data[2]=msg->blue_ang_min;
        global_env->yellow_side_goal_data[1]=msg->yellow_ang_max;
        global_env->yellow_side_goal_data[2]=msg->yellow_ang_min;
        if(global_env->teamcolor == "Blue"){
            global_env->home[global_env->RobotNumber].goal_edge.max = msg->yellow_ang_max;
            global_env->home[global_env->RobotNumber].goal_edge.min = msg->yellow_ang_min;
            global_env->home[global_env->RobotNumber].op_goal_edge.max = msg->blue_ang_max;
            global_env->home[global_env->RobotNumber].op_goal_edge.min = msg->blue_ang_min;
        }else if(global_env->teamcolor == "Yellow"){
            global_env->home[global_env->RobotNumber].goal_edge.max = msg->blue_ang_max;
            global_env->home[global_env->RobotNumber].goal_edge.min = msg->blue_ang_min;
            global_env->home[global_env->RobotNumber].op_goal_edge.max = msg->yellow_ang_max;
            global_env->home[global_env->RobotNumber].op_goal_edge.min = msg->yellow_ang_min;
        }
    }
    void subBlackObject(const std_msgs::Int32MultiArray::ConstPtr &msg){

        int All_Line_distance[360];
        int angle[360];
        int angle_apf=0;
        int reg_angle=0;
        NodePtr ob_data;

        for(int i=0; i<360/Blackangle; i++){
            All_Line_distance[i] = msg -> data[i];
            if(angle_apf<360){
                angle_apf=Blackangle+angle_apf;
                reg_angle=angle_apf;
            }else{
                angle_apf=Blackangle+angle_apf;
                reg_angle=angle_apf-360;
            }

            reg_angle=angle_apf;
            angle[i]=reg_angle;
        }


        angle_apf=0;

        int count=0;
        int dis_avg=0;
        int apf_data[3];
        int dis=125;


        for(int i=0; i<360/Blackangle; i++){
            if(abs(All_Line_distance[i]-All_Line_distance[i+1])<=10){
                if(count==0){
                    apf_data[0]=angle[i];
                    dis_avg=dis_avg+All_Line_distance[i];
                    count++;
                }
                else{
                    dis_avg=dis_avg+All_Line_distance[i];
                    count++;
                }
            }else{
                if(count!=0){
                    count++;
                    apf_data[1]=angle[i];
                    apf_data[2]=(dis_avg+All_Line_distance[i])/count;
                    if(apf_data[2]<=dis){
                        ob_data=creat_node(apf_data[0],apf_data[1],apf_data[2]);
                        append_node(ob_data);
                        //printf("angle_start:%d\tangle:%d\tdis:%d\n",apf_data[0],apf_data[1],apf_data[2]);
                        //printf("angle_start:%d\tangle:%d\tdis:%d\n",angle_start[k],angle_end[k],apf_dis[k]);
                        number_obstacle++;
                    }
                    dis_avg=0;
                    count=0;
                }
                else
                {
                    dis_avg=0;
                    count=0;
                }
            }
        }
     
        //display_list(ob_data);
        list_filter(ob_data);
        //display_list(ob_data);

            global_env->global_angle_end.clear();
            global_env->global_angle_start.clear();
            global_env->global_apf_dis.clear();
            std::vector<int>().swap(global_env->global_angle_end);
            std::vector<int>().swap(global_env->global_angle_start);
            std::vector<int>().swap(global_env->global_apf_dis);

            if(number_obstacle==0){
                global_env->global_angle_end.push_back(0);
                global_env->global_angle_start.push_back(0);
                global_env->global_apf_dis.push_back(0);
            }else{
                global_env->global_angle_end.push_back(1);
                global_env->global_angle_start.push_back(1);
                global_env->global_apf_dis.push_back(1);
            }




        list_to_global(ob_data);


        number_obstacle=0;
        freelist();

        int place;
        Range Unscan[3];
        Unscan[0].begin = Scan[0] - (Scan[3]-1);
        Unscan[0].end   = Scan[0] + (Scan[3]-1);
        Unscan[1].begin = Scan[1] - (Scan[4]-1);
        Unscan[1].end   = Scan[1] + (Scan[4]-1);
        Unscan[2].begin = Scan[2] - (Scan[4]-1);
        Unscan[2].end   = Scan[2] + (Scan[4]-1);
        for(int i=0;i<360/Blackangle;i++){
                All_Line_distance[i] = msg -> data[i];
        }

        global_env->mindis[0] = All_Line_distance[0];
        for(int i=1;i<360/Blackangle;i++){
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
        static double Begin_time = 0;
        static double Current_time = 0;
        Current_time = ros::Time::now().toSec();
        if(global_env->gameState==0){
            Begin_time = ros::Time::now().toSec();
            Current_time = ros::Time::now().toSec();
        }
        //printf("global_env->gameState=%d\n",global_env->gameState);
        if((abs(Current_time-Begin_time)<2)&&(global_env->gameState==5)){
           degree_controller=270;
           ignore_limit_angle=40;
           limit_obstacle_dis=200;
           edge_controller = 1;
        }else if((fabs(Current_time-Begin_time)<2.5)&&(global_env->gameState==GameState_FreeKick)){
           //printf("fabs(Current_time-Begin_time)=%f\n",fabs(Current_time-Begin_time));
           degree_controller=270;
           ignore_limit_angle=40;
           limit_obstacle_dis=200;
           edge_controller = 1;
        }else if(roleAry[global_env->RobotNumber]==Role_Test1){
//           printf("test1\n");
//           degree_controller=270;
//           ignore_limit_angle=50;
//           limit_obstacle_dis=200;
//           edge_controller = 1;
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
        if(roleAry[global_env->RobotNumber]==11||roleAry[global_env->RobotNumber]==Role_Support){// if robot is support or newsupport, ignore limit area
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
    void subWhiteObject(const std_msgs::Int32MultiArray::ConstPtr &msg){

        int All_Line_distance[360];
        int angle[360];
        double shortest_distance = 999;
        double final_distance = 0;
        for(int i=0;i<360/Whiteangle;i++){
                All_Line_distance[i] = msg -> data[i];
        }
        int i=0;
        for(i=165/Whiteangle;i<=195/Whiteangle;i++){// only scan back 30 degree
            if(All_Line_distance[i]<shortest_distance){// good distance
                shortest_distance = All_Line_distance[i];
                //printf("final_distance=%f\n",shortest_distance);
            }else{
                //nothing here, skip to next round
            }
        }
        //printf("whiteangle=%d\n",Whiteangle);
        final_distance=shortest_distance/100;
        //printf("final_distance=%f\n",final_distance);
        global_env->Support_WhiteLine_distance = final_distance;


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
