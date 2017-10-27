#include "NodeHandle.hpp"
Client::Client(int argc, char **argv, const char *node_name)
{
    std::cout << "Initializing NodeHandle...\n";
    ros::init(argc, argv, node_name);
    ROS_INFO("Connected to roscore");
}
void Client::ros_comms_init()
{
    nh = new ros::NodeHandle();
    amcl_pose_sub = nh->subscribe("Wait_Minda", 1000, &Client::AmclPoseSub, this);
    reset_command_sub = nh->subscribe("Wait_Minda2", 1000, &Client::ResetCommandSub, this);
    reset_imu_pub = nh->advertise<std_msgs::Int32>("/ResetImuData", 100);
    amcl_init_pub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100);
}
int GetResetCommand(){
    return __reset_command;
}
void AmclPoseSub(geometry_msgs::Pose2D& msg){
    reset_pose.x = msg.x;
    reset_pose.y = msg.y;
};
void ResetCommandSub(std_msgs::Int32& msg){
    __reset_command = msg.data;
};
void ResetImuPub(){
    std_msgs::Int32 reset_command;
    reset_imu_pub.publish(reset_command);
};
void AmclInitPub(){
    geometry_msgs::PoseWithCovarianceStamped init_pose;
    init_pose.header.frame_id = init_pose;
    init_pose.header.stamp =  ros::Time::now();
    init_pose.pose.pose.position.x = reset_pose.x;
    init_pose.pose.pose.position.y = reset_pose.y;
    init_pose.pose.pose.orientation.w = 1;
};