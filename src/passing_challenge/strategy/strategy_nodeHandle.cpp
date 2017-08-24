#include "strategy_nodeHandle.h"
Strategy_nodeHandle::Strategy_nodeHandle()
{
	this->status = 0;
	this->level = 0;
	this->environment.robot.pos.x = 0;
	this->environment.robot.pos.y = 0;
	this->environment.robot.pos.z = 0;
	this->environment.robot.pos.angle = 0;
	this->environment.robot.ball.distance = 0;
	this->environment.robot.ball.angle = 0;
	this->environment.robot.v_x = 0;
	this->environment.robot.v_y = 0;
	this->environment.robot.v_yaw = 0;
	this->environment.robot.shoot = 0;
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
    loadParam();
	this->vision_sub = n->subscribe<vision::Object>(vision_topic_name, 1000, &Strategy_nodeHandle::visionCallback, this);
	this->IMU_sub = n->subscribe<imu_3d::inertia>(IMU_topic_name, 1000, &Strategy_nodeHandle::IMUCallback, this);
	this->motionFB_sub = n->subscribe<geometry_msgs::Twist>(motionFB_topic_name, 1000, &Strategy_nodeHandle::motionFBCallback, this);
	this->localization_sub = n->subscribe<geometry_msgs::PoseWithCovarianceStamped>(localization_topic_name, 1000, &Strategy_nodeHandle::localizationCallback, this);
	this->level_sub = n->subscribe<std_msgs::Int32>(level_topic_name, 1000, &Strategy_nodeHandle::levelCallback, this);
	this->loadParam_sub = n->subscribe<std_msgs::Int32>(loadParam_topic_name, 1000, &Strategy_nodeHandle::loadParamCallback, this);
	this->status_sub = n->subscribe<std_msgs::Int32>(status_topic_name, 1000, &Strategy_nodeHandle::statusCallback, this);
	this->motion_pub = n->advertise<geometry_msgs::Twist>(motion_topic_name, 1000);
	this->shoot_pub = n->advertise<std_msgs::Int32>(shoot_topic_name, 1000);

}

void Strategy_nodeHandle::visionCallback(const vision::Object::ConstPtr &vision_msg)
{
	this->environment.robot.ball.distance = vision_msg->ball_dis/100.0;
	this->environment.robot.ball.angle = (vision_msg->ball_ang <= 180)? vision_msg->ball_ang : vision_msg->ball_ang - 360;
}

void Strategy_nodeHandle::IMUCallback(const imu_3d::inertia::ConstPtr &imu_msg)
{
	double imu_yaw = (imu_msg->yaw >= (M_PI))? (imu_msg->yaw*180/M_PI-360) : (imu_msg->yaw*180/M_PI); 
	this->environment.robot.pos.z = (-1)*imu_yaw; 
}

void Strategy_nodeHandle::motionFBCallback(const geometry_msgs::Twist::ConstPtr &motionFB_msg)
{
	this->environment.robot.pos.angle = motionFB_msg->angular.z;
}

void Strategy_nodeHandle::localizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &localization_msg)
{
	this->environment.robot.pos.x = localization_msg->pose.pose.position.x;
	this->environment.robot.pos.y = localization_msg->pose.pose.position.y;
	//env.robot.pos.z = localization_msg->pose.pose.position.z;
}

void Strategy_nodeHandle::levelCallback(const std_msgs::Int32::ConstPtr &level_msg)
{
	this->level = level_msg->data;
}

void Strategy_nodeHandle::loadParamCallback(const std_msgs::Int32::ConstPtr &loadParam_msg)
{
	if(loadParam_msg->data==1){
		loadParam();
	}
}

void Strategy_nodeHandle::statusCallback(const std_msgs::Int32::ConstPtr &status_msg)
{
	this->status = status_msg->data;
}

void Strategy_nodeHandle::loadParam()
{
	this->n->getParam("/hold_ball_distance", this->environment.param.hold_ball_distance);
	this->n->getParam("/hold_ball_angle", this->environment.param.hold_ball_angle);
   	this->environment.param.loss_ball_distance = this->environment.param.hold_ball_distance + 0.1;
	this->environment.param.loss_ball_angle = this->environment.param.hold_ball_angle + 10;
	this->n->getParam("/speed_const", this->environment.param.speed_const);
	this->n->getParam("/yaw_const", this->environment.param.yaw_const);
	this->n->getParam("/shoot_const", this->environment.param.shoot_const);
	this->n->getParam("/target_ball_distance", this->environment.param.target_ball_distance);
	this->n->getParam("/target_goal_distance", this->environment.param.target_goal_distance);
	printf("speeed const: %f\n",this->environment.param.speed_const);
	printf("yaw const: %f\n",this->environment.param.yaw_const);
	printf("shoot const: %f\n",this->environment.param.shoot_const);
	printf("target ball distance: %f\n",this->environment.param.target_ball_distance);
	printf("target goal distance: %f\n",this->environment.param.target_goal_distance);
	sleep(2);
}

void Strategy_nodeHandle::pub(int speed)
{
	motion.linear.x = 0;
	motion.linear.y = 0;
	motion.angular.z = 0;
	motion_pub.publish(motion);
	shoot.data = 0;
	shoot_pub.publish(shoot);
}

void Strategy_nodeHandle::pub()
{
	motion.linear.x = environment.robot.v_x;
	motion.linear.y = environment.robot.v_y;
	motion.angular.z = environment.robot.v_yaw;
	motion_pub.publish(motion);
	shoot.data = environment.robot.shoot;
	shoot_pub.publish(shoot);

}
