#include "strategy_nodeHandle.h"
Strategy_nodeHandle::Strategy_nodeHandle()
{

}

Strategy_nodeHandle::~Strategy_nodeHandle()
{

}

void Strategy_nodeHandle::init(int argc, char** argv)
{

	std::cout << "Strategy_nodeHandle::init\n";
	for(int i=0; i<argc; i++){
		std::cout << argv[i] << std::endl;	
	}	
	
	ros::init(argc, argv, "Passing_Challenge_Strategy");
	n = new ros::NodeHandle();
	this->vision_sub = n->subscribe<vision::Object>(vision_topic_name, 1000, &Strategy_nodeHandle::visionCallback, this);
	this->IMU_sub = n->subscribe<imu_3d::inertia>(IMU_topic_name, 1000, &Strategy_nodeHandle::IMUCallback, this);
	this->motionFB_sub = n->subscribe<geometry_msgs::Twist>(motionFB_topic_name, 1000, &Strategy_nodeHandle::motionFBCallback, this);
	this->localization_sub = n->subscribe<geometry_msgs::PoseWithCovarianceStamped>(localization_topic_name, 1000, &Strategy_nodeHandle::localizationCallback, this);
	this->level_sub = n->subscribe<std_msgs::Int32>(level_topic_name, 1000, &Strategy_nodeHandle::levelCallback, this);
	this->loadParam_sub = n->subscribe<std_msgs::Bool>(loadParam_topic_name, 1000, &Strategy_nodeHandle::loadParamCallback, this);

}

void Strategy_nodeHandle::visionCallback(const vision::Object::ConstPtr &vision_msg)
{
	this->env.ball.pos.distance = vision_msg->ball_dis;
	this->env.ball.pos.angle = vision_msg->ball_ang;
}

void Strategy_nodeHandle::IMUCallback(const imu_3d::inertia::ConstPtr &imu_msg)
{
	this->env.robot.pos.z = imu_msg->yaw;
}

void Strategy_nodeHandle::motionFBCallback(const geometry_msgs::Twist::ConstPtr &motionFB_msg)
{
	this->env.robot.pos.angle = motionFB_msg->angular.z;
}

void Strategy_nodeHandle::localizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &localization_msg)
{
	this->env.robot.pos.x = localization_msg->pose.pose.position.x;
	this->env.robot.pos.y = localization_msg->pose.pose.position.y;
	//env.robot.pos.z = localization_msg->pose.pose.position.z;
}

void Strategy_nodeHandle::levelCallback(const std_msgs::Int32::ConstPtr &level_msg)
{
	this->level = level_msg->data;
}

void Strategy_nodeHandle::loadParamCallback(const std_msgs::Bool::ConstPtr &loadParam_msg)
{
	if(loadParam_msg->data==true){
		loadParam();
	}
}

void Strategy_nodeHandle::statusCallback(const std_msgs::Int32::ConstPtr &status_msg)
{
	this->status = status_msg->data;
}

void Strategy_nodeHandle::loadParam()
{

}
