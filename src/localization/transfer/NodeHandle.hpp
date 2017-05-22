///=========include==========
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/LaserScan.h"
#include "imu_3d/inertia.h"
#include "geometry_msgs/Twist.h"
///=========define==========
#define deg2rad 3.1415926/180
#define WhiteLine_Topic "/vision/whiteRealDis"
///=========include & define end==========
class Client{
private:
    ros::NodeHandle *nh;
    ros::Subscriber WhiteLine_sub;
    std_msgs::Int32MultiArray WhiteLine;
    ros::Publisher LaserScan_pub;
    int WhiteAngle;
    int All_Line_distance[360];
    
public:
    Client(int argc, char** argv,const char* node_name);
    ~Client() {}
    void ros_comms_init();
//    topic publish or subscriber
    void whiteline_sub(const std_msgs::Int32MultiArray::ConstPtr &msg);
    void whiteline_pub();
    void imu_sub(const imu_3d::inertia &msg);
    void motorFB_sub(const geometry_msgs::Twist &msg);
    void estimateFB_pub();
//    param
    void loadParam(ros::NodeHandle *nh);
    ros::NodeHandle* getNodeHandle(){return nh;}
};
