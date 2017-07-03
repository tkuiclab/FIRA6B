#include <iostream>
#include <boost/bind.hpp>
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"


#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include "re_status_plugin/ModelCommand.h"
#include <stdio.h>
#include "gazebo_msgs/GetModelState.h"
#include <math/gzmath.hh>


/// This example creates a ModelPlugin, and applies a force to a box to move
/// it alone the ground plane.
namespace gazebo
{
class MYCAR : public ModelPlugin
{
private:
    double left_vel,right_vel;
    math::Pose pos;
    ros::NodeHandle *rosNode;

    ros::ServiceServer cmdServ;
    ros::CallbackQueue srvQueue;
    boost::thread callbackQueeuThread_srv;

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
        std::cout << "in Model" << std::endl;

        left_vel = 0;
        right_vel = 0;

        // Store the pointer to the model
        this->model = _parent;
	// Load parameters for this plugin
      	if (this->LoadParams(_sdf))
      	{
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        	this->updateConnection = event::Events::ConnectWorldUpdateStart(
            boost::bind(&MYCAR::OnUpdate, this));
      	}

        std::cout << "gazebo Model Load finidh" << std::endl;

        //start====about ros====//
        if (!ros::isInitialized()) {
            gzerr << "Not loading plugin since ROS hasn't been "
                << "properly initialized.  Try starting gazebo with ros plugin:\n"
                << "  gazebo -s libgazebo_ros_api_plugin.so\n";
            return;
        }


        this->rosNode = new ros::NodeHandle("");

        ROS_INFO("create FIRA_Sim/<model_name>/status");
        std::cout << "create FIRA_Sim/"+ this->model->GetName() +"/status service" << std::endl;

        ros::AdvertiseServiceOptions servO =
                ros::AdvertiseServiceOptions::create<re_status_plugin::ModelCommand>
                ("FIRA_Sim/"+ this->model->GetName() +"/status", boost::bind(&MYCAR::cmdCallBack, this, _1, _2),
                ros::VoidPtr(), &this->srvQueue);
        this->cmdServ = this->rosNode->advertiseService(servO);

        this->callbackQueeuThread_srv =
                boost::thread(boost::bind(&MYCAR::SrvQueueThread, this));

        std::cout << "create FIRA_Sim/"+ this->model->GetName() +"/status service Finish" << std::endl;

        //end====about ros====//

    }

    public: bool LoadParams(sdf::ElementPtr _sdf) 
    {
      if (this->FindJointByParam(_sdf, this->left_wheel_joint_,
                             "left_wheel_hinge") &&
          this->FindJointByParam(_sdf, this->right_wheel_joint_,
                             "right_wheel_hinge"))
        return true;
      else
        return false;
    }

    public: bool FindJointByParam(sdf::ElementPtr _sdf,
                                  physics::JointPtr &_joint,
                                  std::string _param)
    {
      if (!_sdf->HasElement(_param))
      {
        gzerr << "param [" << _param << "] not found\n";
        return false;
      }
      else
      {
        _joint = this->model->GetJoint(
          _sdf->GetElement(_param)->Get<std::string>());

        if (!_joint)
        {
          gzerr << "joint by name ["
                << _sdf->GetElement(_param)->Get<std::string>()
                << "] not found in model\n";
          return false;
        }
      }
      return true;
    }
    void SrvQueueThread() {
        std::cout << "in SrvQueueThread()" << std::endl;

        while (this->rosNode->ok()) {
            this->srvQueue.callAvailable();
        }
    }

    bool cmdCallBack(re_status_plugin::ModelCommand::Request  &req,
              re_status_plugin::ModelCommand::Response &res)
    {
        //res.sum = req.a + req.b;
        left_vel = req.re_leftVel;
        right_vel = req.re_rightVel;

        res.re_leftVel = this->left_wheel_joint_->GetVelocity(0);
        res.re_rightVel = this->left_wheel_joint_->GetVelocity(0);
        res.x = pos.pos.x;
        res.y = pos.pos.y;
        res.z = pos.pos.z;
        res.yaw = pos.rot.GetYaw();
        res.srvEcho = "Get Cmd";
        ROS_INFO("request: left_vel=%lf, right_vel=%lf", req.re_leftVel, req.re_rightVel);
        //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
        return true;
     }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      //this->left_wheel_joint_->SetForce(0, 0.2);
      //this->right_wheel_joint_->SetForce(0, -0.2);

      double desire_leftVel = left_vel;
      double desire_righttVel = right_vel;
      double wheel_radius = 0.1;
      double l_w = desire_leftVel/wheel_radius;
      double r_w = desire_righttVel/wheel_radius;
      pos = this->model->GetWorldPose();
      
      this->left_wheel_joint_->SetMaxForce(0,4);
      this->right_wheel_joint_->SetMaxForce(0,4);
      
      this->left_wheel_joint_->SetVelocity(0,l_w);
      this->right_wheel_joint_->SetVelocity(0,r_w);
      
      //printf("model_position_x = %f\n", pos.pos.x);
      //printf("model_position_y = %f\n", pos.pos.y);
      //printf("model_position_z = %f\n", pos.pos.z);
      //printf("model_position_roll = %f\n", pos.rot.GetRoll());
      //printf("model_position_pitch = %f\n", pos.rot.GetPitch());
      //printf("model_position_yaw = %f\n", pos.rot.GetYaw());


      //printf("left_wheel_joint_=%lf\n",this->left_wheel_joint_->GetVelocity(0));
      //printf("right_wheel_joint_=%lf\n",this->right_wheel_joint_->GetVelocity(0));
    }


  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MYCAR)
}
