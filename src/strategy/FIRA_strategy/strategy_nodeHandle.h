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
//BlackObject_distance
#define  BlackObject_Topic "/vision/BlackRealDis"
//one_Robot speed
#define Robot_Topic_Speed "/motion/cmd_vel"
//robot suffix
#define Robot_Position_Topic_Suffix "/Strategy/WorldMap/RobotPos"
#define Robot_Role_Topic_Suffix "/Strategy/Coach/role"
#define RobotSpeed_Topic_Suffix "/Strategy/PathPlan/RobotSpeed"

#define Node_Name "PersonalStrategy"

//RobotNumber
#define RobotNumber_Topic "/FIRA/RobotNumber"

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

    void setOpponent(bool inBool){opponent = inBool;}

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
    ros::Subscriber robot_1_role_sub;
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

    /// load param begin
    std::vector<double> SPlanning_Velocity;
    std::vector<double> Distance_Settings;
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

    void robot_1_role_fun(const std_msgs::Int32::ConstPtr &msg){
        roleAry[0] = msg->data;
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
                double ang1 = msg->blue_ang1;
                double ang2 = msg->blue_ang2;
                if(ang1 == ang2){
                    return;
                }
                ang1 = two_point_angle_fix(ang1);
                ang2 = two_point_angle_fix(ang2);
                if(ang1 < 61){
                    ang1 = ang2 + 180;
                }else if(ang2 > -62){
                    ang2 = ang1 - 180;
                }
                ang1 = angle_fix(ang1);
                ang2 = angle_fix(ang2);
                global_env->home[global_env->RobotNumber].opgoal_edge.distance = msg->blue_dis;
                global_env->home[global_env->RobotNumber].opgoal_edge.angle_1 = ang1;
                global_env->home[global_env->RobotNumber].opgoal_edge.angle_2 = ang2;

            }else if(global_env->teamcolor == "Yellow" && msg->yellow_ang1 != msg->yellow_ang2){
                double ang1 = msg->yellow_ang1;
                double ang2 = msg->yellow_ang2;
                if(ang1 == ang2){
                    return;
                }
                ang1 = two_point_angle_fix(ang1);
                ang2 = two_point_angle_fix(ang2);
                if(ang1 < 61){
                    ang1 = ang2 + 180;
                }else if(ang2 > -62){
                    ang2 = ang1 - 180;
                }
                ang1 = angle_fix(ang1);
                ang2 = angle_fix(ang2);
                global_env->home[global_env->RobotNumber].opgoal_edge.distance = msg->yellow_dis;
                global_env->home[global_env->RobotNumber].opgoal_edge.angle_1 = ang1;
                global_env->home[global_env->RobotNumber].opgoal_edge.angle_2 = ang2;
            }
        }
        //for goalkeeper on 5th robot with ros
        double two_point_angle_fix(double angle){
            if(angle <= 225){
                angle = 45 - angle;
            }else{
                angle = 405 - angle;
            }
            return angle;
        }
        double angle_fix(double angle){
            if(angle > 180){
                angle = angle -360;
            }else if(angle < -180){
                angle = angle =360;
            }
            return angle;
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
//ROS_INFO("blackangle[0]=%d,mindis[0]=%d\n",global_env->blackangle[0],global_env->mindis[0]);
        /*}else counter++;*/
   //        std_msgs::Int32MultiArray blackdis;
   //        int blackpop=0;
   //        for(int i=0; i<360/Blackangle; i++){
   //            blackdis.data.push_back(msg->data[i]);
   //        }
   //        for(int i = 0; i < 360/Blackangle; i++){
   //            blackpop = blackdis.data.back();
   //            blackdis.data.pop_back();
   //            printf("%d=%d\n",i,blackpop);
   //        }
   //===================================================================
//           int All_Line_distance[360];
//           int All_Line_angle[360];
//           int place;
//           int middle;
//           int middle_place;
//           int count=1;
//           int lastCount = 0;
//           for(int i=0; i<360/Blackangle; i++){
//               All_Line_distance[i] = msg -> data[i];
////               ROS_INFO("%d=%d\n",i, All_Line_distance[i] );
//           }
//           global_env->mindis = All_Line_distance[0];

//          for(int i=1 ; i<360/Blackangle ; i++){
//               if(All_Line_distance[i] < global_env->mindis){
//                    lastCount = 0;
//                    count = 1;
//                    global_env->mindis = All_Line_distance[i];
////                    ROS_INFO("count=%d\n",count);
//               }else if(All_Line_distance[i] == All_Line_distance[i-1]){
//                    count++;
//                    if(count > lastCount){
//                        lastCount = count;
//                        place = i;
//                    }
////                    ROS_INFO("count=%d\n",count);
//               }else{
//                   if(count > lastCount){
//                       lastCount = count;
//                       place = i;
//                   }
//               }
//               count = 1;
//          }

//          middle = lastCount/2;
//          middle_place = place - middle;

//          for(int i=0 ; i<360/Blackangle ; i++){
//             All_Line_angle[i] = i*Blackangle;
//          }
//          if(All_Line_angle[middle_place] <= 180){
//               global_env->blackangle = All_Line_angle[middle_place];
//          }else if(All_Line_angle[middle_place] > 180){
//               global_env->blackangle = - (360 - All_Line_angle[middle_place]);
//          }

////       ROS_INFO("min=%d\t",global_env->mindis);
////       ROS_INFO("%d\t",place);
////       ROS_INFO("middle_place=%d\t",middle_place);
////       ROS_INFO("lastCount=%d\n",lastCount);
////       ROS_INFO("%d",global_env->blackangle);
//===============================================================================================================
/*        int j=0,k=0;
        bool boolha=0;
        int meanvalue=0;
        int All_Line_distance[360];
        int All_Line_angle[360];
        int temp = 0;
        for(int i=0;i<20;i++){
            Save[i].counter=1;
        }
        for(int i=0; i<360/Blackangle; i++){
           All_Line_distance[i] = msg -> data[i];
//           ROS_INFO("%d=%d\n",i, All_Line_distance[i] );
             }
        for(int i=0;i<(360/Blackangle)-1;i++){
            if(All_Line_distance[i]<200){
                if(Save[j].counter==1){
                    Save[j].distance=All_Line_distance[i];
                    Save[j].location=i;
                }
                //fabs(All_Line_distance[i]-All_Line_distance[i+1])
                meanvalue += All_Line_distance[i]-All_Line_distance[i+1];
                if(abs(meanvalue)<=3){
                    Save[j].counter++;
                    boolha=1;
                }else if(boolha == 1){
                    j++;
                    meanvalue=0;
                    boolha=0;
                }else if(fabs(All_Line_distance[i]-All_Line_distance[i+1]) > 3){
                    meanvalue=0;
                }
            }
        }


              for(int i=0;i<j;i++){
                  if(Save[i].counter >= 3){
                      New_Save[k].location = Save[i].location;
                      New_Save[k].distance = Save[i].distance;
                      New_Save[k].counter = Save[i].counter;
                      k++;
                  }
              }
              for(int i=0; i<k ; i++){
                  for(int j=i; j<k; j++){
                      if(New_Save[j].distance < New_Save[i].distance){
                          temp = New_Save[j].distance;
                          New_Save[j].distance = New_Save[i].distance;
                          New_Save[i].distance = temp;

                          temp = New_Save[j].location;
                          New_Save[j].location = New_Save[i].location;
                          New_Save[i].location = temp;

                          temp = New_Save[j].counter;
                          New_Save[j].counter = New_Save[i].counter;
                          New_Save[i].counter = temp;
                      }
                   }
              }
              for(int i=0; i<k-1; i++){
                if(New_Save[i].distance == New_Save[i+1].distance){
                  if(New_Save[i].counter < New_Save[i+1].counter){
                     temp = New_Save[i].location;
                     New_Save[i].location= New_Save[i+1].location;
                     New_Save[i+1].location = temp;

                     temp = New_Save[i].counter;
                     New_Save[i].counter= New_Save[i+1].counter;
                     New_Save[i+1].counter = temp;
                  }
                }
              }
              for(int i=0; i<k; i++){
                   global_env->mindis[i] = New_Save[i].distance;
                   New_Save[i].middle = New_Save[i].counter/2;
                   New_Save[i].middle_place = New_Save[i].location + New_Save[i].middle ;
              }
              for(int i=0 ; i<360/Blackangle ; i++){
                  All_Line_angle[i] = i*Blackangle;
              }
              for(int i=0; i<k; i++){
                  if(All_Line_angle[New_Save[i].middle_place] <= 180){
                       global_env->blackangle[i] = All_Line_angle[New_Save[i].middle_place];
                  }else if(All_Line_angle[New_Save[i].middle_place] > 180){
                       global_env->blackangle[i] = - (360 - All_Line_angle[New_Save[i].middle_place]);
                  }
              }*/
//======================================================================================================================//

//              for(int i=0;i<j;i++){
//                  ROS_INFO("min[%d]=%d\t loaction[%d]=%d\t counter[%d]=%d\n",i,Save[i].distance,i,Save[i].location,i,Save[i].counter);
//              }
//              for(int i=0;i<k;i++){
//                  ROS_INFO("New.distance[%d]=%d\t New.location[%d]=%d\t New_Save[%d].counter=%d\n",i,New_Save[i].distance,i,New_Save[i].location,i,New_Save[i].counter);
//                   ROS_INFO("global_env->mindis[%d]=%d\t global_env->blackangle[%d]=%d\n \n",i,global_env->mindis[i],i,global_env->blackangle[i]);
//              }
    





    }
};

#endif /* NODE_HPP_ */
