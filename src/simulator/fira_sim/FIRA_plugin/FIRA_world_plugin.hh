#ifndef _GAZEBO_FIRA_world_plugin_HH_
#define _GAZEBO_FIRA_world_plugin_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include "fira_sim/WorldPluginSrv.h"
#include "std_msgs/String.h"

#include "ros/ros.h"

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

        physics::ModelPtr FIRA_ground;
        math::Pose ballpos;
        math::Pose robot_yellow_1pos;
        math::Pose robot_yellow_2pos;
        math::Pose robot_yellow_3pos;
        math::Pose robot_blue_1pos;
        math::Pose robot_blue_2pos;
        math::Pose robot_blue_3pos;

        ros::Publisher score_pub;

        //for service
        ros::CallbackQueue srvQueue;
        boost::thread callbackQueeuThread_srv;

        double current_time;
        double pre_out_judge_time;
        double pre_dribbling_time;

        int yellow_sc,blue_sc;


        bool out_judge;      //true:


        ros::Subscriber lasttouch;

        int gameState;
        int whosball;
        std::string whos;
        std::string stateHistory;
        public: FIRA_world_plugin();

        public:virtual ~FIRA_world_plugin();
        public:virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);
        void SrvQueueThread();
        void sub_pub_init();
        void checkPos_Model();
        private:virtual void OnUpdate();
        void whoball(const std_msgs::String::ConstPtr& msg);
        std::string execSysCmd(const char* cmd);
        void reset();
        void FIRArobot_reset();
        void goalcheck();
        void throw_in_check();
        bool user_cmd(fira_sim::WorldPluginSrv::Request &req, fira_sim::WorldPluginSrv::Response &res);
        //====Dribbling====
        void Dribbling();
        bool check_dribbling;
        double distance_br;
        double vector_br_x;
        double vector_br_y;
        double dribbling_x,dribbling_y;
        int sign_x;
        int sign_y;
        double angle_br;
        bool check_reset;
    };
}
#endif
