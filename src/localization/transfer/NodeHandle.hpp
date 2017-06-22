///=========include==========
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "math.h"
#include "sensor_msgs/LaserScan.h"
#include "imu_3d/inertia.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
///=========define==========
#define deg2rad 3.1415926/180
#define rad2deg 180/3.1415926
#define WhiteLine_Topic "/vision/whiteRealDis"
#define motorFB_Topic "/motorFB"
#define imu3d_Topic "/imu_3d"
///=========include & define end==========
class Client{
private:
    ros::NodeHandle *nh;
    ros::Subscriber WhiteLine_sub;
    ros::Subscriber MotorFB_sub;
    ros::Subscriber Imu3d_sub;
    std_msgs::Int32MultiArray WhiteLine;
    ros::Publisher LaserScan_pub;
    ros::Publisher Odom_pub ;
    tf::StampedTransform tf_map_to_odom_;
    tf::TransformBroadcaster odom_broadcaster;
    int WhiteAngle;
    int All_Line_distance[360];
    double imu3d;
    double FB_x;
    double FB_y;
    double new_FB_x;
    double new_FB_y;    
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
    void odom_tf_pub();
    ros::NodeHandle* getNodeHandle(){return nh;}
};
