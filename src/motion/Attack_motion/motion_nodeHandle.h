#ifndef motion_nodeHandle_H
#define motion_nodeHandle_H
/*********************
 ** Include system
 *********************/
#include <iostream>
/*********************
 ** Include ROS
 *********************/
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

/*********************
 ** Include library
 *********************/
#include "../common/motor_data.h"
/*********************
 ** Define 
 *********************/
#define odometry_topic_name "/odom"
#define motion_topic_name "/motion"
#define shoot_topic_name "/shoot"
#define DEBUG
class Motion_nodeHandle{
public:
	Motion_nodeHandle(int argc, char **argv);
	virtual ~Motion_nodeHandle();
	
private:
//	const double m1_Angle = -M_PI/3;
//	const double m2_Angle =  M_PI/3;
//	const double m3_Angle = -M_PI;
//	const double robot_radius = 0.15;
//	const double wheel_radius = 0.00508;

	ros::NodeHandle *n;
	ros::Publisher odom_pub;
	ros::Subscriber motion_sub;
	ros::Subscriber shoot_sub;
	command *nodeCMD;
	motor_feedback *nodeFB;
	//double x_speed;
	//double y_speed;
	//double yaw_speed;
	//int shoot_power;
private:
	void init(int argc, char **argv);
	void motionCallback(const geometry_msgs::Twist::ConstPtr &);
	void shootCallback(const std_msgs::Int32::ConstPtr &);
//	void inverseKinematics();
public:
	command* getMotion();
	motor_feedback* getMotor_feedback();
	void clearshoot();
};
#endif
