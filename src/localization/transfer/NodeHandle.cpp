#include "NodeHandle.hpp"
Client::Client(int argc, char** argv,const char* node_name){
    std::cout << "Initializing Node...\n";
    ros::init(argc,argv,node_name);
    ROS_INFO("Connected to roscore");
}

void Client::ros_comms_init(){
    nh = new ros::NodeHandle();
    WhiteLine_sub = nh->subscribe(WhiteLine_Topic,1000,&Client::whiteline_sub,this);
    LaserScan_pub = nh->advertise<sensor_msgs::LaserScan>("/scan",1000);
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
    printf("success sub the imu_3d value");
}
void Client::motorFB_sub(const geometry_msgs::Twist &msg){
    printf("success sub the motorFB value");
}
void Client::estimateFB_pub(){
    printf("success pub the estimateFB value");
}
void Client::whiteline_pub(){
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
        if(All_Line_distance[i]==999)
            scan.ranges[i] = 999;
//        printf("i=%d,value=%10lf\n",i,scan.ranges[i]);
    }
    LaserScan_pub.publish(scan);
}
