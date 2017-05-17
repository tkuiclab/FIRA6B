#include "motion_nodeHandle.h"
Motion_nodeHandle::Motion_nodeHandle(int argc, char **argv)
{
	this->nodeCMD = new command;
	this->nodeFB = new motor_feedback;
	this->nodeCMD->x_speed = new double;
	this->nodeCMD->y_speed = new double;
	this->nodeCMD->yaw_speed = new double;
	this->nodeCMD->shoot_power = new int;
	//x_speed = 0;
	//y_speed = 0;
	//yaw_speed = 0;
	//shoot_power = 0;
	init(argc, argv);
}

Motion_nodeHandle::~Motion_nodeHandle()
{
	ros::shutdown();
#ifdef DEBUG
	std::cout << "~Motion_nodeHandle(DEBUG)\n";
#endif
	//delete nodeCMD;
	//delete nodeFB;
}

void Motion_nodeHandle::init(int argc, char **argv)
{
	ros::init(argc, argv, "Attack_motion");
#ifdef DEBUG
	std::cout << "nodeHandle init(DEBUG)\n";
	std::cout << "PATH= " << *argv << std::endl;
#endif
	n = new ros::NodeHandle();
	odom_pub = n->advertise<nav_msgs::Odometry>(odometry_topic_name,1000);
	motion_sub = n->subscribe<geometry_msgs::Twist>(motion_topic_name, 1000, &Motion_nodeHandle::motionCallback, this);
	shoot_sub = n->subscribe<std_msgs::Int32>(shoot_topic_name, 1000, &Motion_nodeHandle::shootCallback, this);
}

void Motion_nodeHandle::motionCallback(const geometry_msgs::Twist::ConstPtr &motion_msg)
{
	//this->x_speed = motion_msg->linear.x;
	//this->y_speed = motion_msg->linear.y;
	//this->yaw_speed = motion_msg->angular.z;
	*(double*)(this->nodeCMD->x_speed) = motion_msg->linear.x;
	*(double*)(this->nodeCMD->y_speed) = motion_msg->linear.y;
	*(double*)(this->nodeCMD->yaw_speed) = motion_msg->angular.z;
#ifdef DEBUG
	std::cout << "motionCallback(DEBUG)\n";
	std::cout << "X axis speed: " << *(double*)(this->nodeCMD->x_speed) << std::endl;
	std::cout << "Y axis speed: " << *(double*)(this->nodeCMD->y_speed) << std::endl;
	std::cout << "yaw speed: " << *(double*)(this->nodeCMD->yaw_speed) << std::endl;
	std::cout << std::endl;
#endif
}


void Motion_nodeHandle::shootCallback(const std_msgs::Int32::ConstPtr &shoot_msg)
{
	//this->shoot_power = shoot_msg->data;
	*(int*)nodeCMD->shoot_power = shoot_msg->data;
#ifdef DEBUG
	std::cout << "shootCallback(DEBUG)\n";
	std::cout << "shoot power: " << *(int*)(nodeCMD->shoot_power) << std::endl;
#endif
}

command* Motion_nodeHandle::getMotion()
{
	//this->nodeCMD->x_speed = x_speed;
	//this->nodeCMD->y_speed = y_speed;
	//this->nodeCMD->yaw_speed = yaw_speed;
	//this->nodeCMD->shoot_power = shoot_power;
	return this->nodeCMD;
}

void Motion_nodeHandle::clearshoot()
{
	*(int*)(this->nodeCMD->shoot_power) = 0;
}
