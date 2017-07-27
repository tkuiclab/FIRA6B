#ifndef strategy_nodeHandle_H
#define strategy_nodeHandle_H
/****************************
 *		Include				*
 ****************************/
#include <iostream>
#include <cmath>
/****************************
 *		Include ROS			*
 ****************************/
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "vision/Object.h"
#include "imu_3d/inertia.h"
/****************************
 *		Include Library		*
 ****************************/
#include "Env.h"
/****************************
 *	Define Topic Nmae   *
 ****************************/
#define vision_topic_name "/vision/object"
#define IMU_topic_name "/imu_3d"
#define motionFB_topic_name "/motion/motionFB"
#define localization_topic_name "/amcl_pose"
#define level_topic_name "/level"
#define loadParam_topic_name "/loadParam"
#define status_topic_name "/status"
#define motion_topic_name "/motion/cmd_vel"
#define shoot_topic_name "/motion/shoot"

#define DEBUG
class Strategy_nodeHandle{
public:
	Strategy_nodeHandle();
	~Strategy_nodeHandle();
protected:
	void init(int, char**);
	Environment environment;
	int level;
	int status;
private:
	ros::NodeHandle *n;
	ros::Subscriber vision_sub;
	ros::Subscriber IMU_sub;
	ros::Subscriber motionFB_sub;
	ros::Subscriber localization_sub;
	ros::Subscriber level_sub;
	ros::Subscriber loadParam_sub;
	ros::Subscriber status_sub;
	ros::Publisher	motion_pub;
	ros::Publisher	shoot_pub;

	geometry_msgs::Twist motion;
	std_msgs::Int32 shoot;
private:
	void visionCallback(const vision::Object::ConstPtr &);
	void IMUCallback(const imu_3d::inertia::ConstPtr &);
	void motionFBCallback(const geometry_msgs::Twist::ConstPtr &);
	void localizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);
	void levelCallback(const std_msgs::Int32::ConstPtr &);
	void loadParamCallback(const std_msgs::Bool::ConstPtr &);
	void statusCallback(const std_msgs::Int32::ConstPtr &);
	void loadParam();
public:
	Environment* getEnv(){return &environment;}
	void pub(int);
	void pub();
};
#endif
