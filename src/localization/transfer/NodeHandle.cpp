#include "NodeHandle.hpp"
Client::Client(int argc, char** argv,const char* node_name){
    std::cout << "Initializing NodeHandle...\n";
    ros::init(argc,argv,node_name);
    ROS_INFO("Connected to roscore");
}
void Client::ros_comms_init(){
    nh = new ros::NodeHandle();
    WhiteLine_sub = nh->subscribe(WhiteLine_Topic,1000,&Client::whiteline_sub,this);
    LaserScan_pub = nh->advertise<sensor_msgs::LaserScan>("/scan",1000);
    MotorFB_sub = nh->subscribe(motorFB_Topic,1000,&Client::motorFB_sub,this);
    Imu3d_sub = nh->subscribe(imu3d_Topic,10000,&Client::imu_sub,this);
    Odom_pub = nh->advertise<nav_msgs::Odometry>("/odom", 50);
    // Initialpose_pub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1000);
}
void Client::loadParam(ros::NodeHandle* nh){
    nh->getParam("/FIRA/whiteline/angle",WhiteAngle);
}
void Client::whiteline_sub(const std_msgs::Int32MultiArray::ConstPtr &msg){
    for(int i=0; i<360/WhiteAngle; i++){
        All_Line_distance[i] = msg -> data[i];
//        printf("i=%d,value=%d\n",i,All_Line_distance[i]);
    }
}
void Client::imu_sub(const imu_3d::inertia &msg){
    imu3d = msg.yaw;
    // imu3d = imu3d*rad2deg;
    // printf("yaw = %lf\n",imu3d);
}
void Client::motorFB_sub(const geometry_msgs::Twist &msg){
    // printf("motorFB_X = %f\n",msg.linear.x);
    // printf("motorFB_y = %f\n",msg.linear.y);
    FB_x = msg.linear.x;
    FB_y = msg.linear.y;
    estimateFB_pub();
}
void Client::estimateFB_pub(){
    printf("success pub the estimateFB value\n");
    new_FB_x = cos(imu3d)*FB_x + sin(imu3d)*FB_y;
    new_FB_y = -sin(imu3d)*FB_x + cos(imu3d)*FB_y;
    printf("FB_x = %f\n",FB_x);
    printf("FB_y = %f\n",FB_y);   
    printf("new_FB_x = %f\n",new_FB_x);
    printf("new_FB_y = %f\n",new_FB_y);
    printf("imu3d = %lf\n",imu3d);
}
void Client::whiteline_pub(){
    if(WhiteAngle==0)
        WhiteAngle = 1;
    int num_readings = 360/WhiteAngle;
    // printf("%d\n",num_readings);
    int laser_frequency = 30; // fps is not sure
    sensor_msgs::LaserScan scan;
    ros::Time scan_time = ros::Time::now();
    scan.header.frame_id = "laser_frame";
    scan.header.stamp = scan_time;
    scan.angle_min = -180*deg2rad;
    scan.angle_max = 180*deg2rad;
    scan.angle_increment = WhiteAngle*deg2rad;
    // printf("%f\n",WhiteAngle*deg2rad);
    scan.time_increment = (1 / (float)laser_frequency) / (float)(num_readings);
    scan.scan_time = (1 / (float)laser_frequency);
    scan.range_min = 0.0;
    scan.range_max = 999;
    scan.ranges.resize(num_readings);
    for(int i = 0; i < num_readings; ++i){
        scan.ranges[i] = All_Line_distance[i]/100.0;
        // if(All_Line_distance[i]==999)
        //     scan.ranges[i] = -1.0;
//        printf("i=%d,value=%10lf\n",i,scan.ranges[i]);
    }
    LaserScan_pub.publish(scan);
}
void Client::odom_tf_pub(){
    ros::Time current_time, last_time;
    current_time = ros::Time::now();  
    last_time = ros::Time::now();
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = new_FB_x;
    odom_trans.transform.translation.y = new_FB_y;
    odom_trans.transform.translation.z = 0;
    // odom_trans.transform.rotation = imu3d;
    odom_trans.transform.rotation.x = 0;
    odom_trans.transform.rotation.y = 0;
    odom_trans.transform.rotation.z = 0;
    odom_trans.transform.rotation.w = 1;
    odom_broadcaster.sendTransform(odom_trans);
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
 
    //set the position
    odom.pose.pose.position.x = new_FB_x;
    odom.pose.pose.position.y = new_FB_y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;
    Odom_pub.publish(odom);
    last_time = current_time;
}
void Client::initialpose_pub(){
    ros::Time current_time = ros::Time::now();  
    geometry_msgs::PoseWithCovarianceStamped init_pose;
    init_pose.header.stamp = current_time;
    init_pose.header.frame_id = "initialpose";
    init_pose.pose.pose.position.x = 0;
    init_pose.pose.pose.position.y = 0;
    init_pose.pose.pose.position.z = 0;
    Initialpose_pub.publish(init_pose);
}