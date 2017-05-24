#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
int main(int argc, char** argv)
{
	ros::init(argc, argv, "simulate_strategy");
	ros::NodeHandle n;
	ros::Publisher pub_motion = n.advertise<geometry_msgs::Twist>("/motion",1000);
	ros::Publisher pub_shoot = n.advertise<std_msgs::Int32>("/shoot", 1000);
	double x_speed = 0;
	double y_speed = 0;
	double yaw_speed = 0;
	geometry_msgs::Twist motion_msg;
	std_msgs::Int32 shoot_msg;
	shoot_msg.data=0;
	ros::Rate loop_rate(30);
	int counter = 0;
	while(ros::ok()){
		counter = counter%10;
		if(counter<3){
			x_speed=x_speed+0.5;
			if(x_speed>100)x_speed=0;
		}else if(counter<6){
			y_speed=y_speed+0.3;
			if(y_speed>100)y_speed=0;
			shoot_msg.data = (shoot_msg.data)+1;
			if(shoot_msg.data>100)shoot_msg.data=0;
		}else{
			yaw_speed=yaw_speed+0.1;
			if(yaw_speed>100)yaw_speed=0;
		}
		motion_msg.linear.x = x_speed;
		motion_msg.linear.y = y_speed;
		motion_msg.angular.z = yaw_speed;
		pub_motion.publish(motion_msg);
		pub_shoot.publish(shoot_msg);
		counter++;
		ros::spinOnce();
		loop_rate.sleep();
	}
}
