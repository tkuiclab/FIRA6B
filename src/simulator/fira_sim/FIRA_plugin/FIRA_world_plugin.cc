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
#include "FIRA_world_plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "geometry_msgs/Twist.h"

//#include "fira_sim/RefereeSrv.h"
#include "fira_sim/WorldPluginSrv.h"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/common/Exception.hh"
#include "FIRA_status_plugin/ModelMsg.h"

#define SIGN(A) ((A)>=0?1:-1)
//#define Topic_Name "/FIRA/Simulaotr/WorldPlugin"
#define SrvName "/FIRA/Simulaotr/WorldPluginSrv"
#define RobotSpeed_Topic_Suffix "/RobotSpeed"

#define GameState_Play          0
#define GameState_Halt          1
#define GameState_FreeKick      2
#define GameState_PenaltyKick   3
#define GameState_Kickoff       4
#define GameState_ThrowIn       5
#define GameState_CornerKick    6
#define GameState_GoalKick      7

#define Halt_time   2.0

#define BlueTeam 0
#define YellowTeam 1


using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(FIRA_world_plugin)

FIRA_world_plugin::FIRA_world_plugin() : WorldPlugin()
{
}

FIRA_world_plugin::~FIRA_world_plugin()
{
}

void FIRA_world_plugin::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        mPtr = _parent;

        rosNode = new ros::NodeHandle("FIRA_WorldPlugin");

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

        out_judge = true ;  //init outside judge

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&FIRA_world_plugin::OnUpdate,this));

        lasttouch = rosNode->subscribe("/Last_TouchBall", 1000 ,&FIRA_world_plugin::whoball,this);

        std::cout << "ballPtr=" << ballPtr << std::endl;
//        std::cout << "Robot_Yellow_1=" << Robot_Yellow_1 << std::endl;
    }

void FIRA_world_plugin::SrvQueueThread() {
        std::cout << "in SrvQueueThread()" << std::endl;
        while (this->rosNode->ok()) {
            this->srvQueue.callAvailable();
        }
    }

void FIRA_world_plugin::whoball(const std_msgs::String::ConstPtr& msg){

    whos = msg->data.c_str();

    std::string who = msg->data.c_str();
    who="R2";
    if(who == "R1" || who == "R2" || who == "R3"){  //B team
        whosball = BlueTeam;
    }else if(who == "Opt_R1" || who == "Opt_R2" || who == "Opt_R3"){ //Y team
        whosball = YellowTeam;
    }
}


void FIRA_world_plugin::checkPos_Model(){
        FIRA_ground->SetWorldPose(math::Pose(math::Vector3(0,0,0),math::Vector3(0,0,0)));
        blue_goal->SetWorldPose(math::Pose(math::Vector3(3.35,-0.92,0),math::Vector3(0,0,3.14)));
        yellow_goal->SetWorldPose(math::Pose(math::Vector3(-3.35,-0.48,0),math::Vector3(0,0,0)));

        if(Robot_Blue_1==NULL||Robot_Blue_2==NULL||Robot_Blue_3==NULL||Robot_Yellow_1==NULL||Robot_Yellow_2==NULL||Robot_Yellow_3==NULL){
            Robot_Blue_1 = mPtr->GetModel("R1");
            Robot_Blue_2 = mPtr->GetModel("R2");
            Robot_Blue_3 = mPtr->GetModel("R3");
            Robot_Yellow_1 = mPtr->GetModel("Opt_R1");
            Robot_Yellow_2 = mPtr->GetModel("Opt_R2");
            Robot_Yellow_3 = mPtr->GetModel("Opt_R3");
        }
    }

void FIRA_world_plugin::OnUpdate()
    {
        checkPos_Model();

        //====Get World Pose====//
        ballpos = ballPtr->GetWorldPose();
        if(current_time > 6 ||check_reset==true){
            robot_blue_2pos = Robot_Blue_2->GetWorldPose();
        }
        //***TIME COUNTER
        std::string time = gazebo::common::Time::GetWallTimeAsISOString();
        common::Time ct = mPtr->GetRealTime();
        current_time = ct.Double();
        //for(int i = 0 ; i <=5 ;i++) checkrobotpose(Robot_Blue_+i);

        //-----
        if(out_judge){
            if(  (current_time - pre_out_judge_time) > Halt_time){
                if(ballpos.pos.x > -3 || ballpos.pos.x < 3){
                    gameState = GameState_ThrowIn;
                }else if(ballpos.pos.x<-3 && whosball == BlueTeam){
                    gameState = GameState_GoalKick;
                }else if(ballpos.pos.x<-3 && whosball == YellowTeam){
                    gameState = GameState_CornerKick;
                }else if(ballpos.pos.x>3 && whosball == YellowTeam){
                    gameState = GameState_GoalKick;
                }else if(ballpos.pos.x>3 && whosball == BlueTeam){
                    gameState = GameState_CornerKick;
                }
            }
        }

        if(ballpos.pos.x > -3 && ballpos.pos.x < 3 && ballpos.pos.y > -2 && ballpos.pos.y <2){  //enable outside judge
            out_judge = true ;
        }

        /*if( current_time - start_time > 15){  //game time
            reset();
        }*/

        //------corner
        goalcheck();

        //---------side of the ball
//        throw_in_check();

        //---------Dribbling
//        if(  (current_time - pre_dribbling_time) > 0.05)Dribbling();
        Dribbling();
}

std::string FIRA_world_plugin::execSysCmd(const char* cmd) {
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

void FIRA_world_plugin::reset(){
        //====kill all *strategy* process====//
//        std::string pidResponse = execSysCmd("ps -A | grep strategy | awk '{print $1}' | xargs");
//        std::cout << "killpid Response = " <<  pidResponse << std::endl;
//        std::string killStr = "kill "+pidResponse;
//        std::string killResponse = execSysCmd(killStr.c_str());
//        std::cout << "killResponse = " <<  killResponse << std::endl;

        //====speed reset====//
//        geometry_msgs::Twist speedMsg;
//        speedMsg.linear.x = 0;
//        speedMsg.linear.y = 0;
//        speedMsg.angular.z= 0;

//        robotSpeedPub_blue_1.publish(speedMsg);
//        robotSpeedPub_blue_2.publish(speedMsg);
//        robotSpeedPub_blue_3.publish(speedMsg);
//        robotSpeedPub_yellow_1.publish(speedMsg);
//        robotSpeedPub_yellow_2.publish(speedMsg);
//        robotSpeedPub_yellow_3.publish(speedMsg);

        FIRArobot_reset();

        //====Score reset====//
        blue_sc = 0;
        yellow_sc = 0;



    }
void FIRA_world_plugin::FIRArobot_reset(){
    //====model reset====//
    physics::Model_V modelList = mPtr->GetModels();
    for(int i =0 ;i < modelList.size();i++){
        physics::ModelPtr tPtr = modelList[i];
        tPtr->Reset();
    }

    check_dribbling =false;
    check_reset = true;
    //wait 0.5seconds
    sleep(0.01);
}

void FIRA_world_plugin::goalcheck(){

        if(ballpos.pos.x < -3 && out_judge == true){   //left side
            if(ballpos.pos.x > -3.4 && ballpos.pos.y > -0.5 && ballpos.pos.y <0.5){
                ROS_INFO("GOAL!!!!!!!!");
                blue_sc++;
                //ballPtr->SetWorldPose(math::Pose(math::Vector3(0,0,0.1),math::Quaternion(0,0,0)));
                //Robot_Blue_1->SetWorldPose(math::Pose(math::Vector3(0,1,0.1),math::Quaternion(0,0,0)));
                //Robot_Blue_2->SetWorldPose(math::Pose(math::Vector3(0,1,0.1),math::Quaternion(0,0,0)));

                FIRArobot_reset();
            }else{
//                if(ballpos.pos.y<0){
//                    ROS_INFO("out");
//                    out_judge = false;
//                    gameState = GameState_Halt;
//                    pre_out_judge_time = current_time;
//                    if(whosball == YellowTeam){
//                        stateHistory = "["+boost::lexical_cast<std::string>(current_time)+"]"+"outside,Y team's ball";
//                        FIRArobot_reset();
//                        ballPtr->SetWorldPose(math::Pose(math::Vector3(-3.5,-1.9,0.1),math::Quaternion(0,0,0)));
//                        Robot_Blue_2->SetWorldPose(math::Pose(math::Vector3(-4,-1.9,0.1),math::Quaternion(0,0,0)));
//                    }else if(whosball == BlueTeam){
//                        stateHistory = "["+boost::lexical_cast<std::string>(current_time)+"]"+"outside,B team's ball";
//                        FIRArobot_reset();
//                        ballPtr->SetWorldPose(math::Pose(math::Vector3(-2.25,0,0.1),math::Quaternion(0,0,0)));
//                        Robot_Yellow_1->SetWorldPose(math::Pose(math::Vector3(-3,0,0.1),math::Quaternion(0,0,0)));
//                    }

//                }else{
//                    ROS_INFO("out");
//                    out_judge = false;
//                    gameState = GameState_Halt;
//                    pre_out_judge_time = current_time;
//                    if(whosball == YellowTeam){
//                        stateHistory = "["+boost::lexical_cast<std::string>(current_time)+"]"+"outside,Y team's ball";
//                        FIRArobot_reset();
//                        ballPtr->SetWorldPose(math::Pose(math::Vector3(-2.25,0,0.1),math::Quaternion(0,0,0)));
//                        Robot_Blue_2->SetWorldPose(math::Pose(math::Vector3(-4,1.9,0.1),math::Quaternion(0,0,0)));
//                    }else if(whosball == BlueTeam){
//                        stateHistory = "["+boost::lexical_cast<std::string>(current_time)+"]"+"outside,B team's ball";
//                        FIRArobot_reset();
//                        ballPtr->SetWorldPose(math::Pose(math::Vector3(-3.5,1.9,0.1),math::Quaternion(0,0,0)));
//                        Robot_Yellow_1->SetWorldPose(math::Pose(math::Vector3(-3,0,0.1),math::Quaternion(0,0,0)));
//                    }
//                }
            }
        }
        if(ballpos.pos.x > 3 && out_judge == true){     //right side
            if(ballpos.pos.x < 3.4 && ballpos.pos.y > -0.5 && ballpos.pos.y <0.5){
                ROS_INFO("GOAL!!!!!!!!");
                yellow_sc++;
                //ballPtr->SetWorldPose(math::Pose(math::Vector3(0,0,0.1),math::Quaternion(0,0,0)));
                FIRArobot_reset();
            }else{
//                if(ballpos.pos.y<0){
//                    ROS_INFO("out");
//                    out_judge = false;
//                    gameState = GameState_Halt;
//                    pre_out_judge_time = current_time;
//                    if(whosball == BlueTeam){
//                        stateHistory = "["+boost::lexical_cast<std::string>(current_time)+"]"+"outside,Y team's ball";
//                        FIRArobot_reset();
//                        ballPtr->SetWorldPose(math::Pose(math::Vector3(3.5,-1.9,0.1),math::Quaternion(0,0,0)));
//                        Robot_Yellow_2->SetWorldPose(math::Pose(math::Vector3(4,-1.9,0.1),math::Quaternion(0,0,3.14)));
//                    }else if(whosball == YellowTeam){
//                        stateHistory = "["+boost::lexical_cast<std::string>(current_time)+"]"+"outside,B team's ball";
//                        FIRArobot_reset();
//                        ballPtr->SetWorldPose(math::Pose(math::Vector3(2.25,0,0.1),math::Quaternion(0,0,0)));
//                        Robot_Blue_1->SetWorldPose(math::Pose(math::Vector3(3,0,0.1),math::Quaternion(0,0,3.14)));
//                    }
//                }else{
//                    ROS_INFO("out");
//                    out_judge = false;
//                    gameState = GameState_Halt;
//                    pre_out_judge_time = current_time;
//                    if(whosball == BlueTeam){
//                        stateHistory = "["+boost::lexical_cast<std::string>(current_time)+"]"+"outside,Y team's ball";
//                        FIRArobot_reset();
//                        ballPtr->SetWorldPose(math::Pose(math::Vector3(3.5,1.9,0.1),math::Quaternion(0,0,0)));
//                        Robot_Yellow_2->SetWorldPose(math::Pose(math::Vector3(4,1.9,0.1),math::Quaternion(0,0,3.14)));
//                    }else if(whosball == YellowTeam){
//                        stateHistory = "["+boost::lexical_cast<std::string>(current_time)+"]"+"outside,B team's ball";
//                        FIRArobot_reset();
//                        ballPtr->SetWorldPose(math::Pose(math::Vector3(2.25,0,0.1),math::Quaternion(0,0,0)));
//                        Robot_Blue_1->SetWorldPose(math::Pose(math::Vector3(3,0,0.1),math::Quaternion(0,0,3.14)));
//                    }
//                }
            }
        }
    }

void FIRA_world_plugin::throw_in_check(){
        if(ballpos.pos.y > 2 && ballpos.pos.x < 3 && ballpos.pos.x > -3 && out_judge == true){//up side
            ROS_INFO("out");
            try{
                FIRArobot_reset();
                ballPtr->SetWorldPose(math::Pose(math::Vector3(ballpos.pos.x,2,0.1),math::Quaternion(0,0,0)));
                out_judge = false;
                gameState = GameState_Halt;
                pre_out_judge_time = current_time;
                if(whosball == BlueTeam){
                    stateHistory = "["+boost::lexical_cast<std::string>(current_time)+"]"+"outside,Y team's ball";
                    Robot_Yellow_2->SetWorldPose(math::Pose(math::Vector3(ballpos.pos.x,2.5,0.1),math::Quaternion(0,0,-1.57)));
                }else if(whosball == YellowTeam){
                    stateHistory = "["+boost::lexical_cast<std::string>(current_time)+"]"+"outside,B team's ball";
                    Robot_Blue_2->SetWorldPose(math::Pose(math::Vector3(ballpos.pos.x,2.5,0.1),math::Quaternion(0,0,-1.57)));
                }
            }catch(common::Exception e){
                std::cout << "Error" << e.GetErrorStr() << std::endl;
            }
        }
        if(ballpos.pos.y < -2 && ballpos.pos.x < 3 && ballpos.pos.x > -3 && out_judge == true){//down side
            ROS_INFO("out");
            FIRArobot_reset();
            ballPtr->SetWorldPose(math::Pose(math::Vector3(ballpos.pos.x,-2,0.1),math::Quaternion(0,0,0)));
            out_judge = false;
            gameState = GameState_Halt;
            pre_out_judge_time = current_time;
            if(whosball == BlueTeam){
                stateHistory = "["+boost::lexical_cast<std::string>(current_time)+"]"+"outside,Y team's ball";                
                Robot_Yellow_2->SetWorldPose(math::Pose(math::Vector3(ballpos.pos.x,-2.5,0.1),math::Quaternion(0,0,1.57)));
            }else if(whosball == YellowTeam){
                stateHistory = "["+boost::lexical_cast<std::string>(current_time)+"]"+"outside,B team's ball";                
                Robot_Blue_2->SetWorldPose(math::Pose(math::Vector3(ballpos.pos.x,-2.5,0.1),math::Quaternion(0,0,1.57)));
            }
        }
    }

    //void user_cmd(const std_msgs::String::ConstPtr& inMsg)
bool FIRA_world_plugin::user_cmd(fira_sim::WorldPluginSrv::Request &req, fira_sim::WorldPluginSrv::Response &res){

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

void FIRA_world_plugin::Dribbling(){
    pre_dribbling_time = current_time;
    distance_br = hypot(ballpos.pos.x-robot_blue_2pos.pos.x,ballpos.pos.y-robot_blue_2pos.pos.y);
    vector_br_x = ballpos.pos.x-robot_blue_2pos.pos.x;
    vector_br_y = ballpos.pos.y-robot_blue_2pos.pos.y;
    sign_x=1;
    sign_y=1;
    if(vector_br_y!=0||vector_br_x!=0){
        angle_br = (atan2(vector_br_y, vector_br_x) *180/M_PI)-robot_blue_2pos.rot.GetYaw()*180/M_PI;

        if(angle_br > 180)angle_br = angle_br - 360;
        else if(angle_br < -180)angle_br = angle_br + 360;

        if((distance_br<0.33 && (fabs(angle_br)<20))||check_dribbling==true){
            check_dribbling=true;
            if(robot_blue_2pos.rot.GetYaw()*180/M_PI>90)sign_x=(-1);
            else if(robot_blue_2pos.rot.GetYaw()*180/M_PI<-90) {sign_x=(-1);sign_y=(-1);}
            else if(robot_blue_2pos.rot.GetYaw()*180/M_PI<0 && robot_blue_2pos.rot.GetYaw()*180/M_PI>-90)sign_y=(-1);

            dribbling_x=robot_blue_2pos.pos.x+sign_x*0.32*fabs(cos(robot_blue_2pos.rot.GetYaw()));
            dribbling_y=robot_blue_2pos.pos.y+sign_y*0.32*fabs(sin(robot_blue_2pos.rot.GetYaw()));
            ballPtr->SetWorldPose(math::Pose(math::Vector3(dribbling_x,dribbling_y,0.1),math::Quaternion(0,0,0)));
        }
    }
}
