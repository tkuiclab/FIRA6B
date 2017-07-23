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
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <time.h>
///=========define==========
#define deg2rad 3.1415926/180
#define rad2deg 180/3.1415926
#define WhiteLine_Topic "/vision/WhiteRealDis"
#define motorFB_Topic "/motion/motionFB"
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
    ros::Publisher Odom_pub;
    ros::Publisher Initialpose_pub;
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
    void whiteline_sub(const std_msgs::Int32MultiArray::ConstPtr &msg){
        for(int i=0; i<360/WhiteAngle; i++)
            All_Line_distance[i] = msg -> data[i];
    }
    void whiteline_pub();
    void imu_sub(const imu_3d::inertia &msg){imu3d = msg.yaw;}
    void motorFB_sub(const geometry_msgs::Twist &msg){FB_x = msg.linear.x;FB_y = msg.linear.y;}
    void odom_tf_pub();
    void initialpose_pub();
//    param
    void loadParam(ros::NodeHandle *nh);
    ros::NodeHandle* getNodeHandle(){return nh;}
};
