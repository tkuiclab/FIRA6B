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
    WhiteLine_sub = nh->subscribe(WhiteLine_Topic, 1, &Client::whiteline_sub, this);
    LaserScan_pub = nh->advertise<sensor_msgs::LaserScan>("/scan", 1);
    MotorFB_sub = nh->subscribe(motorFB_Topic, 1, &Client::motorFB_sub, this);
    Imu3d_sub = nh->subscribe(imu3d_Topic, 1, &Client::imu_sub, this);
    Odom_pub = nh->advertise<nav_msgs::Odometry>("/odom", 50);
}
void Client::loadParam(ros::NodeHandle *nh)
{
    nh->getParam("/FIRA/HSV/white/angle", WhiteAngle);
}
void Client::whiteline_pub()
{
    if (WhiteAngle == 0)
        WhiteAngle = 1;
    int num_readings = 360 / WhiteAngle;
    int laser_frequency = 90; // vision fps
    sensor_msgs::LaserScan scan;
    ros::Time scan_time = ros::Time::now();
    scan.header.frame_id = "laser_frame";
    scan.header.stamp = scan_time;
    scan.angle_min = -180 * deg2rad;
    scan.angle_max = 180 * deg2rad;
    scan.angle_increment = WhiteAngle * deg2rad;
    scan.time_increment = (1 / (float)laser_frequency) / (float)(num_readings);
    scan.scan_time = (1 / (float)laser_frequency);
    scan.range_min = 0.0;
    scan.range_max = 999;
    scan.ranges.resize(num_readings);
    // for(int i = 0; i < num_readings; ++i){
    //     scan.ranges[i] = All_Line_distance[i]/100.0;
    // }
    int j = 0;
    for (int i = 90; i < num_readings; ++i)
    {
        scan.ranges[j] = All_Line_distance[i] / 100.0;
        j++;
    }
    for (int i = 0; i < 90; ++i)
    {
        scan.ranges[j] = All_Line_distance[i] / 100.0;
        j++;
    }
    LaserScan_pub.publish(scan);
}
void Client::odom_tf_pub()
{
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    // geometry_msgs::TransformStamped odom_trans;
    // odom_trans.header.stamp = current_time;
    // odom_trans.header.frame_id = "odom";
    // odom_trans.child_frame_id = "base_link";
    // static double last_FB_x = FB_x;
    // static double last_FB_y = FB_y;
    // static double map_x = FB_x;
    // static double map_y = FB_y;
    // double delta_x,delta_y;
    // if(fabs(FB_x - last_FB_x)>0.1)
    //     FB_x = last_FB_x;
    // if(fabs(FB_y - last_FB_y)>0.1)
    //     FB_y = last_FB_y;
    // delta_x = FB_x - last_FB_x;
    // delta_y = FB_y - last_FB_y;
    // last_FB_x = FB_x;
    // last_FB_y = FB_y;
    // map_x += (delta_x * cos(-imu3d) - delta_y * sin(-imu3d));
    // map_y += (delta_x * sin(-imu3d) + delta_y * cos(-imu3d));
    // // imu direction is nagitive for quaternion
    // odom_trans.transform.translation.x = map_x;
    // odom_trans.transform.translation.y = map_y;
    // odom_trans.transform.translation.z = 0;
    // odom_trans.transform.rotation.w = cos(0)*cos(0)*cos(-imu3d/2)+sin(0)*sin(0)*sin(-imu3d/2);
    // odom_trans.transform.rotation.x = sin(0)*cos(0)*cos(-imu3d/2)-cos(0)*sin(0)*sin(-imu3d/2);
    // odom_trans.transform.rotation.y = cos(0)*sin(0)*cos(-imu3d/2)+sin(0)*cos(0)*sin(-imu3d/2);
    // odom_trans.transform.rotation.z = cos(0)*cos(0)*sin(-imu3d/2)-sin(0)*sin(0)*cos(-imu3d/2);
    // odom_broadcaster.sendTransform(odom_trans);

    if (dt<0.0005)
        ;
    else
    {
        nav_msgs::Odometry odom;
        static double last_FB_x = FB_x;
        static double last_FB_y = FB_y;
        static double map_x = FB_x;
        static double map_y = FB_y;
        static double last_imu = imu3d;
        double delta_x, delta_y;
        if (fabs(FB_x - last_FB_x) / dt > 300) //  Detection the odom error
            FB_x = last_FB_x;                  // let velocity > 6 m/s is error signal
        if (fabs(FB_y - last_FB_y)/dt > 300)
            FB_y = last_FB_y;
        delta_x = FB_x - last_FB_x;
        delta_y = FB_y - last_FB_y;
        last_FB_x = FB_x;
        last_FB_y = FB_y;
        map_x += (delta_x * cos(-imu3d) - delta_y * sin(-imu3d));
        map_y += (delta_x * sin(-imu3d) + delta_y * cos(-imu3d));
        //  (-) imu   imu's angular is not fit in normal axis
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = map_x;
        odom.pose.pose.position.y = map_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.w = cos(0) * cos(0) * cos(-imu3d / 2) + sin(0) * sin(0) * sin(-imu3d / 2);
        odom.pose.pose.orientation.x = sin(0) * cos(0) * cos(-imu3d / 2) - cos(0) * sin(0) * sin(-imu3d / 2);
        odom.pose.pose.orientation.y = cos(0) * sin(0) * cos(-imu3d / 2) + sin(0) * cos(0) * sin(-imu3d / 2);
        odom.pose.pose.orientation.z = cos(0) * cos(0) * sin(-imu3d / 2) - sin(0) * sin(0) * cos(-imu3d / 2);
        odom.pose.covariance[0] = 1e-3;
        odom.pose.covariance[7] = 1e-3;
        odom.pose.covariance[14] = 1e-3;
        odom.pose.covariance[21] = 1e-2;
        odom.pose.covariance[28] = 1e-2;
        odom.pose.covariance[35] = 1e-2;
        odom.twist.twist.linear.x = delta_x / dt;
        odom.twist.twist.linear.y = delta_y / dt;
        odom.twist.twist.angular.z = (-1) * (imu3d - last_imu) / dt;
        // printf("dx=%lf\tdy=%lf\tdz=%lf\n",delta_x,delta_y,(-1) * (imu3d - last_imu));
        if(odom.twist.twist.linear.x != 0 || odom.twist.twist.linear.y != 0 || odom.twist.twist.angular.z != 0)
            printf("x=%lf\ty=%lf\tz=%lf\n",odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.angular.z);

        last_imu = imu3d;
        Odom_pub.publish(odom);
    }
    last_time = current_time;
}
void Client::initialpose_pub()
{
    ros::Time current_time = ros::Time::now();
    geometry_msgs::PoseWithCovarianceStamped init_pose;
    init_pose.header.stamp = current_time;
    init_pose.header.frame_id = "initialpose";
    init_pose.pose.pose.position.x = 0;
    init_pose.pose.pose.position.y = 0;
    init_pose.pose.pose.position.z = 0;
    Initialpose_pub.publish(init_pose);
}
