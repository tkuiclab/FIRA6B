#include <iostream>
#include <boost/bind.hpp>
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"


#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>


#include "FIRA_status_plugin/ModelMsg.h"
#include <stdio.h>
#include "gazebo_msgs/GetModelState.h"
#include <math/gzmath.hh>

#define PUB_REFIX "/FIRA/Strategy/WorldMap/"

/// This example creates a ModelPlugin, and applies a force to a box to move
/// it alone the ground plane.
namespace gazebo
{
class Obj_Pub : public ModelPlugin
{
private:
    double left_vel,right_vel;
    math::Pose pos;
    ros::NodeHandle *rosNode;

    ros::ServiceServer cmdServ;
    ros::CallbackQueue srvQueue;
    boost::thread callbackQueeuThread_srv;
    ros::Publisher status_pub;


    // Pointer to the model
    physics::ModelPtr model;

    //physics::WorldPtr world;
    //gazebo_msgs::GetModelState getmodelstate;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    private: physics::JointPtr left_wheel_joint_;
    private: physics::JointPtr right_wheel_joint_;

public:

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {


        // Store the pointer to the model
        this->model = _parent;
        std::cout << this->model->GetName() << std::endl;

        this->rosNode = new ros::NodeHandle("");

        //Ros publish

        status_pub = rosNode->advertise<FIRA_status_plugin::ModelMsg>( PUB_REFIX + this->model->GetName(),1000);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Obj_Pub::OnUpdate, this));


        //std::cout << "gazebo Model Load finidh" << std::endl;

        //start====about ros====//
        if (!ros::isInitialized()) {
            gzerr << "Not loading plugin since ROS hasn't been "
                << "properly initialized.  Try starting gazebo with ros plugin:\n"
                << "  gazebo -s libgazebo_ros_api_plugin.so\n";
            return;
        }
        //end====about ros====//

    }

    // Called by the world update start event
    public: void OnUpdate()
    {

      pos = this->model->GetWorldPose();

      FIRA_status_plugin::ModelMsg msg;

      msg.x = pos.pos.x;
      msg.y = pos.pos.y;
      msg.z = pos.pos.z;
      msg.yaw=pos.rot.GetYaw();

      msg.vel_x = this->model->GetLink("link")->GetWorldLinearVel().x;
      msg.vel_y = this->model->GetLink("link")->GetWorldLinearVel().y;

      status_pub.publish(msg);


    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Obj_Pub)
}
