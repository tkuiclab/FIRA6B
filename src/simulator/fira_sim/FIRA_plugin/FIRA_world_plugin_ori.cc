/**
 * @file FIRA_world_plugin.cc
 *
 * @brief world plugin for gazebo
 *
 * @date Auguest 2014
 * @author 連振宇,黃文鴻,林建銘  Chen,Kung-Han
 * @version 1.0
 * @update 2014/08/03    Version 1.0
 * @detail
        Outside: Halt Game ->   2s
                Get last touch from ball ->
                 Throw-in or CornerKick or GoalKick & Judge whos ball　＆ ->
                 Show info  (  [00:00:00] (Outside_TouchLine) - youbot0 outside, B Team's Ball) ->
                 Move Robot & ball (move robot[1] behind 50cm from ball) ->
                 Show info  (  [00:00:00] (Throw-in) - youbot0 throw-in) ->

                 Wait....->
                 (After ball in field)
                 Play Game





 **/
#include <iostream>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
//#include "fira_sim/RefereeSrv.h"
#include "fira_sim/WorldPluginSrv.h"
#include "gazebo/physics/PhysicsTypes.hh"

//#define Topic_Name "/FIRA/Simulaotr/WorldPlugin"
#define SrvName "/FIRA/Simulaotr/WorldPluginSrv"
#define RobotSpeed_Topic_Prefix "/youbot"
#define RobotSpeed_Topic_Suffix "/RobotSpeed"


#define GameState_Play          0
#define GameState_Halt          1
#define GameState_FreeKick      2
#define GameState_PenaltyKick   3
#define GameState_FreeBall      4
#define GameState_ThrowIn       5
#define GameState_CornerKick    6
#define GameState_GoalKick      7

#define Halt_time   2.0


namespace gazebo
{


class FIRA_world_plugin : public WorldPlugin
{
    physics::WorldPtr mPtr;
    ros::NodeHandle *rosNode;

    event::ConnectionPtr updateConnection;


    //ros::Subscriber world_sub;
    ros::ServiceServer mServer;

    transport::PublisherPtr factorypub;

    physics::ModelPtr ballPtr; //soccer

    physics::ModelPtr blue_goal;

    physics::ModelPtr yellow_goal;

    physics::ModelPtr Robot_Blue_1;
    physics::ModelPtr Robot_Blue_2;
    physics::ModelPtr Robot_Blue_3;
    physics::ModelPtr Robot_Yellow_1;
    physics::ModelPtr Robot_Yellow_2;
    physics::ModelPtr Robot_Yellow_3;

    ros::Publisher robotSpeedPub_blue_1;
    ros::Publisher robotSpeedPub_blue_2;
    ros::Publisher robotSpeedPub_blue_3;
    ros::Publisher robotSpeedPub_yellow_1;
    ros::Publisher robotSpeedPub_yellow_2;
    ros::Publisher robotSpeedPub_yellow_3;

    physics::ModelPtr FIRA_ground;
    math::Pose FIRA_G;
    math::Pose ballpos;

    ros::Publisher score_pub;

    //for service
    ros::CallbackQueue srvQueue;
    boost::thread callbackQueeuThread_srv;

    double current_time;
    double pre_out_judge_time;

    int yellow_sc,blue_sc;


    int out_judge;


    ros::Subscriber lasttouch;

    int gameState;
    int whosball;
    std::string whos;
    std::string stateHistory;


    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        mPtr = _parent;

        rosNode = new ros::NodeHandle("FIRA_WorldPlugin");

        //sub_pub_init();

        std::string robotSpeed_prefix = RobotSpeed_Topic_Prefix;
        std::string robotSpeed_suffix = RobotSpeed_Topic_Suffix;

        robotSpeedPub_blue_1 = rosNode->advertise<geometry_msgs::Twist>(robotSpeed_prefix+"0"+robotSpeed_suffix,1000);
        robotSpeedPub_blue_2 = rosNode->advertise<geometry_msgs::Twist>(robotSpeed_prefix+"1"+robotSpeed_suffix,1000);
        robotSpeedPub_blue_3 = rosNode->advertise<geometry_msgs::Twist>(robotSpeed_prefix+"2"+robotSpeed_suffix,1000);
        robotSpeedPub_yellow_1 = rosNode->advertise<geometry_msgs::Twist>(robotSpeed_prefix+"3"+robotSpeed_suffix,1000);
        robotSpeedPub_yellow_2 = rosNode->advertise<geometry_msgs::Twist>(robotSpeed_prefix+"4"+robotSpeed_suffix,1000);
        robotSpeedPub_yellow_3 = rosNode->advertise<geometry_msgs::Twist>(robotSpeed_prefix+"5"+robotSpeed_suffix,1000);
        //world_sub = rosNode->subscribe(Topic_Name,1000,&FIRA_world_plugin::user_cmd,this);
        // mServer = rosNode->advertiseService(SrvName, &FIRA_world_plugin::user_cmd, this);

        ros::AdvertiseServiceOptions servO =
                       ros::AdvertiseServiceOptions::create<fira_sim::WorldPluginSrv>
                       (SrvName, boost::bind(&FIRA_world_plugin::user_cmd, this, _1, _2),
                       ros::VoidPtr(), &this->srvQueue);

       this->mServer = this->rosNode->advertiseService(servO);

       this->callbackQueeuThread_srv =
               boost::thread(boost::bind(&FIRA_world_plugin::SrvQueueThread, this));

        ballPtr = _parent->GetModel("soccer");
        blue_goal = _parent->GetModel("BlueGoal");
        yellow_goal = _parent->GetModel("YellowGoal");
        FIRA_ground = _parent->GetModel("FIRA_Ground");

        out_judge = 1 ;  //init outside judge


        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&FIRA_world_plugin::OnUpdate,this));


        std::cout << "ballPtr=" << ballPtr << std::endl;
        std::cout << "Robot_Blue_1=" << Robot_Blue_1 << std::endl;
    }

    void SrvQueueThread() {
        std::cout << "in SrvQueueThread()" << std::endl;
        while (this->rosNode->ok()) {
            this->srvQueue.callAvailable();
        }
    }

    void whoball(const std_msgs::String::ConstPtr& msg){

        whos = msg->data.c_str();

        std::string who = msg->data.c_str();
        if(who == "youbot0" || who == "youbot1" || who == "youbot2"){  //B team
            whosball = 0;
        }else if(who == "youbot3" || who == "youbot4" || who == "youbot5"){ //Y team
            whosball = 1;
        }
    }

    void sub_pub_init(){
        std::string robotSpeed_prefix = RobotSpeed_Topic_Prefix;
        std::string robotSpeed_suffix = RobotSpeed_Topic_Suffix;

        robotSpeedPub_blue_1 = rosNode->advertise<geometry_msgs::Twist>(robotSpeed_prefix+"0"+robotSpeed_suffix,1000);
        robotSpeedPub_blue_2 = rosNode->advertise<geometry_msgs::Twist>(robotSpeed_prefix+"1"+robotSpeed_suffix,1000);
        robotSpeedPub_blue_3 = rosNode->advertise<geometry_msgs::Twist>(robotSpeed_prefix+"2"+robotSpeed_suffix,1000);
        robotSpeedPub_yellow_1 = rosNode->advertise<geometry_msgs::Twist>(robotSpeed_prefix+"3"+robotSpeed_suffix,1000);
        robotSpeedPub_yellow_2 = rosNode->advertise<geometry_msgs::Twist>(robotSpeed_prefix+"4"+robotSpeed_suffix,1000);
        robotSpeedPub_yellow_3 = rosNode->advertise<geometry_msgs::Twist>(robotSpeed_prefix+"5"+robotSpeed_suffix,1000);
        //world_sub = rosNode->subscribe(Topic_Name,1000,&FIRA_world_plugin::user_cmd,this);


    }
//    void checkrobotpose(physics::ModelPtr _Robot,int num){
//        if(_Robot==NULL){
//            _Robot = mPtr->GetModel("youbot"+num);
//        }
//    }

    void checkPos_Model(){
        FIRA_ground->SetWorldPose(math::Pose(math::Vector3(0,0,0),math::Vector3(0,0,0)));
        blue_goal->SetWorldPose(math::Pose(math::Vector3(3.35,-0.92,0),math::Vector3(0,0,3.14)));
        yellow_goal->SetWorldPose(math::Pose(math::Vector3(-3.35,-0.48,0),math::Vector3(0,0,0)));


        if(Robot_Blue_1==NULL){
            Robot_Blue_1 = mPtr->GetModel("youbot0");
            Robot_Blue_2 = mPtr->GetModel("youbot1");
            Robot_Blue_3 = mPtr->GetModel("youbot2");
            Robot_Yellow_1 = mPtr->GetModel("youbot3");
            Robot_Yellow_2 = mPtr->GetModel("youbot4");
            Robot_Yellow_3 = mPtr->GetModel("youbot5");
        }
    }


    public: void OnUpdate()
    {
        checkPos_Model();
        ballpos = ballPtr->GetWorldPose();
        lasttouch = rosNode->subscribe("/Last_TouchBall", 1000 ,&FIRA_world_plugin::whoball,this);

        //***TIME COUNTER
        std::string time = gazebo::common::Time::GetWallTimeAsISOString();
        common::Time ct = mPtr->GetRealTime();
        current_time = ct.Double();
        //for(int i = 0 ; i <=5 ;i++) checkrobotpose(Robot_Blue_+i);
        if(out_judge == 0){
            if(  (current_time - pre_out_judge_time) > Halt_time){
                if(ballpos.pos.x > -3 && ballpos.pos.x < 3){
                    gameState = GameState_ThrowIn;
                }else if(ballpos.pos.x<-3 && whosball ==0){
                    gameState = GameState_GoalKick;
                }else if(ballpos.pos.x<-3 && whosball ==1){
                    gameState = GameState_CornerKick;
                }else if(ballpos.pos.x>3 && whosball ==0){
                    gameState = GameState_GoalKick;
                }else if(ballpos.pos.x>3 && whosball ==1){
                    gameState = GameState_CornerKick;
                }
            }
        }

        if(ballpos.pos.x > -3 && ballpos.pos.x < 3 && ballpos.pos.y > -2 && ballpos.pos.y <2){  //enable outside judge
            out_judge = 1 ;
        }

        /*if( current_time - start_time > 15){  //game time
            reset();
        }*/

        //------corner
        goalcheck();

        //---------side of the ball
        throw_in_check();
    }

    std::string execSysCmd(const char* cmd) {
        FILE* pipe = popen(cmd, "r");
        if (!pipe) return "ERROR";
        char buffer[128];
        std::string result = "";
        while(!feof(pipe)) {
            if(fgets(buffer, 128, pipe) != NULL)
                result += buffer;
        }
        pclose(pipe);
        return result;
    }

    void resetAllModel(){
        //====model reset====//
        physics::Model_V modelList = mPtr->GetModels();
        for(int i =0 ;i < modelList.size();i++){
            physics::ModelPtr tPtr = modelList[i];
            tPtr->Reset();
        }

    }

    void reset(){
        //====kill all *strategy* process====//
        std::string pidResponse = execSysCmd("ps -A | grep strategy | awk '{print $1}' | xargs");
        std::cout << "killpid Response = " <<  pidResponse << std::endl;
        std::string killStr = "kill "+pidResponse;
        std::string killResponse = execSysCmd(killStr.c_str());
        std::cout << "killResponse = " <<  killResponse << std::endl;

        //====speed reset====//
        geometry_msgs::Twist speedMsg;
        speedMsg.linear.x = 0;
        speedMsg.linear.y = 0;
        speedMsg.angular.z= 0;

        robotSpeedPub_blue_1.publish(speedMsg);
        robotSpeedPub_blue_2.publish(speedMsg);
        robotSpeedPub_blue_3.publish(speedMsg);
        robotSpeedPub_yellow_1.publish(speedMsg);
        robotSpeedPub_yellow_2.publish(speedMsg);
        robotSpeedPub_yellow_3.publish(speedMsg);

        resetAllModel();

        //====Score reset====//
        blue_sc = 0;
        yellow_sc = 0;
    }

    void goalcheck(){
        if(ballpos.pos.x < -3 && out_judge == 1){   //left side
            if(ballpos.pos.x > -3.4 && ballpos.pos.y > -0.5 && ballpos.pos.y <0.5){
                ROS_INFO("GOAL!!!!!!!!");
                blue_sc++;
                //ballPtr->SetWorldPose(math::Pose(math::Vector3(0,0,0.1),math::Quaternion(0,0,0)));
                //Robot_Blue_1->SetWorldPose(math::Pose(math::Vector3(0,1,0.1),math::Quaternion(0,0,0)));
                //Robot_Blue_2->SetWorldPose(math::Pose(math::Vector3(0,1,0.1),math::Quaternion(0,0,0)));

                resetAllModel();
            }else{
                if(ballpos.pos.y<0){
                    ROS_INFO("out");
                    ballPtr->SetWorldPose(math::Pose(math::Vector3(-3.5,-1.9,0.1),math::Quaternion(0,0,0)));
                    out_judge = 0;
                    gameState = GameState_Halt;
                    pre_out_judge_time = current_time;
                    if(whosball==0){
                        stateHistory = boost::lexical_cast<std::string>(current_time)+"outside,B team's ball";
                    }else if(whosball==1){
                        stateHistory = boost::lexical_cast<std::string>(current_time)+"outside,Y team's ball";
                    }

                }else{
                    ROS_INFO("out");
                    ballPtr->SetWorldPose(math::Pose(math::Vector3(-3.5,1.9,0.1),math::Quaternion(0,0,0)));
                    out_judge = 0;
                    gameState = GameState_Halt;
                    pre_out_judge_time = current_time;
                    if(whosball==0){
                        stateHistory = boost::lexical_cast<std::string>(current_time)+"outside,B team's ball";
                    }else if(whosball==1){
                        stateHistory = boost::lexical_cast<std::string>(current_time)+"outside,Y team's ball";
                    }
                }
            }
        }
        if(ballpos.pos.x > 3 && out_judge == 1){     //right side
            if(ballpos.pos.x < 3.4 && ballpos.pos.y > -0.5 && ballpos.pos.y <0.5){
                ROS_INFO("GOAL!!!!!!!!");
                yellow_sc++;
                //ballPtr->SetWorldPose(math::Pose(math::Vector3(0,0,0.1),math::Quaternion(0,0,0)));

                resetAllModel();
            }else{
                if(ballpos.pos.y<0){
                    ROS_INFO("out");
                    ballPtr->SetWorldPose(math::Pose(math::Vector3(3.5,-1.9,0.1),math::Quaternion(0,0,0)));
                    out_judge = 0;
                    gameState = GameState_Halt;
                    pre_out_judge_time = current_time;
                    if(whosball==0){
                        stateHistory = boost::lexical_cast<std::string>(current_time)+"outside,B team's ball";

                    }else if(whosball==1){
                        stateHistory = boost::lexical_cast<std::string>(current_time)+"outside,Y team's ball";
                    }
                }else{
                    ROS_INFO("out");
                    ballPtr->SetWorldPose(math::Pose(math::Vector3(3.5,1.9,0.1),math::Quaternion(0,0,0)));
                    out_judge = 0;
                    gameState = GameState_Halt;
                    pre_out_judge_time = current_time;
                    if(whosball==0){
                        stateHistory = boost::lexical_cast<std::string>(current_time)+"outside,B team's ball";
                    }else if(whosball==1){
                        stateHistory = boost::lexical_cast<std::string>(current_time)+"outside,Y team's ball";
                    }
                }
            }
        }
    }

    void throw_in_check(){
        if(ballpos.pos.y > 2 && ballpos.pos.x < 3 && ballpos.pos.x > -3 && out_judge == 1){
            ROS_INFO("out");
            ballPtr->SetWorldPose(math::Pose(math::Vector3(ballpos.pos.x,2.5,0.1),math::Quaternion(0,0,0)));
            out_judge = 0;
            gameState = GameState_Halt;
            pre_out_judge_time = current_time;
            if(whosball==0){
                stateHistory = boost::lexical_cast<std::string>(current_time)+"outside,Y team's ball";
                Robot_Yellow_2->SetWorldPose(math::Pose(math::Vector3(ballpos.pos.x,3,0.1),math::Quaternion(0,0,-1.57)));
            }else if(whosball==1){
                stateHistory = boost::lexical_cast<std::string>(current_time)+"outside,B team's ball";
                Robot_Blue_2->SetWorldPose(math::Pose(math::Vector3(ballpos.pos.x,3,0.1),math::Quaternion(0,0,-1.57)));
            }
        }
        if(ballpos.pos.y < -2 && ballpos.pos.x < 3 && ballpos.pos.x > -3 && out_judge == 1){
            ROS_INFO("out");
            ballPtr->SetWorldPose(math::Pose(math::Vector3(ballpos.pos.x,-2.5,0.1),math::Quaternion(0,0,0)));

            out_judge = 0;
            gameState = GameState_Halt;
            pre_out_judge_time = current_time;
            if(whosball==0){
                stateHistory = boost::lexical_cast<std::string>(current_time)+"outside,Y team's ball";
                Robot_Yellow_2->SetWorldPose(math::Pose(math::Vector3(ballpos.pos.x,-3,0.1),math::Quaternion(0,0,1.57)));
            }else if(whosball==1){
                stateHistory = boost::lexical_cast<std::string>(current_time)+"outside,B team's ball";
                Robot_Blue_2->SetWorldPose(math::Pose(math::Vector3(ballpos.pos.x,-3,0.1),math::Quaternion(0,0,1.57)));
            }
        }
    }

    //void user_cmd(const std_msgs::String::ConstPtr& inMsg)
    bool user_cmd(fira_sim::WorldPluginSrv::Request &req, fira_sim::WorldPluginSrv::Response &res){

        //ROS_INFO("FIRA_world_plugin say I heard: [%s]", inMsg->data.c_str());

        // mPtr->Reset();
        //if(inMsg->data== "reset"){
        //  ROS_INFO("Reset FIRA World");
        if(req.command == "Reset"){

            ROS_INFO("Reset FIRA World");
            reset();

        }else if(req.command == "Info"){
            res.blueGoal = blue_sc;
            res.yellowGoal = yellow_sc;
            res.gameState = gameState;
            res.whosBall = whos;
            res.stateHistory=stateHistory;

            //Robot_Blue_1->SetWorldPose(math::Pose(math::Vector3(0,0,0),math::Quaternion(0,0,0)));

//            if(Robot_Blue_1==NULL){
//                 Robot_Blue_1=mPtr->GetModel("youbot0");
//            }
//            Robot_Blue_1->SetWorldPose(math::Pose(math::Vector3(0,0,0),math::Quaternion(0,0,0)));
        }
        return true;
    }


};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(FIRA_world_plugin)
}
