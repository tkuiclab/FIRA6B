#include "ros/ros.h"
//#include "move/Num.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include <iostream>
#include <string>
#include "LinuxNetwork.h"
#include "tinyxml2.h"
#include <sstream>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include "../../AllPrjCommon/Utility.h"
#include "../../AllPrjCommon/ROSNameDef.h"

using namespace std;
using namespace Robot;
using namespace tinyxml2;



#define PI 3.14159265

#define ang2rad PI/180
#define VerySmall 0.0001



#define PWM_MIN 37.0
#define PWM_MAX 229.5



//=====ROS define=====
//#define Subtopic_Prefix "FIRA/"
//#define Subtopic_Suffix "/Strategy/PathPlan/RobotSpeed"
#define Node_Name         "motion"


//====motor define=====
#define RobotRadius  0.23              //L
#define WheelRadius  0.0508             //R
#define RPM_Max      6940
#define Gear         26
#define PWM_Range    255.0
#define PWM_Limit_Percent_Min   0.1
#define PWM_Limit_Percent_Max   0.9

#define RPM_Offset  0

//#define DEBUG

const double rad2rpm =  60.0  / (2.0*M_PI);
const double percentRange =  PWM_Limit_Percent_Max - PWM_Limit_Percent_Min;





//====server const=====
const string serverIP = "192.168.2.34";
const int serverPort = 10373;


struct timeval start, end;
double averageTime_sum = 0.0;
int averageTime_count = 0;

LinuxSocket *client;

static bool sendLock = false;

void sTime()
{
    gettimeofday(&start, NULL);
}

double eTime(string preStr){

    gettimeofday(&end, NULL);

    long utime, seconds, useconds;
    double mtime;

    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;



    mtime = (seconds * 1000.0 + useconds/1000.0) + 0.5;
    printf("%s : %02d(s)  = %8.4lf(ms)\n",preStr.c_str(),seconds,mtime);

    return mtime;
}

void averageTime(double iTime_MS){
     if(iTime_MS > 0.0){
        averageTime_sum += iTime_MS;
        averageTime_count++;
        double averageTime = (averageTime_sum) / (double)averageTime_count;
        std::cout << "[average Time = " << averageTime << "]" << std::endl;
    }
}


#ifdef DEBUG
    int old_pwm_1,old_pwm_2,old_pwm_3;
#endif
//input:
//  x: velocity of x (m/s)
//  y: velocity of x (m/s)
//  yaw: velocity of x (rad/s)
//output
//  xml for fpga

string speed2pwm(float v_x,float v_y,float v_yaw,int en){
    //motor speed
int en_1,en_2,en_3;
int dir_1,dir_2,dir_3;
std::ostringstream q;

    float w1,w2,w3;

    int pwm_1,pwm_2,pwm_3;
    float pwmRatio_1,pwmRatio_2,pwmRatio_3;
 

    w1= -0.5*v_x + 0.866*v_y + RobotRadius * v_yaw;
    w2= -0.5*v_x - 0.866*v_y + RobotRadius * v_yaw;
    w3=  v_x + RobotRadius * v_yaw;


    w1 = w1 * (1/WheelRadius)  * Gear ;
    w2 = w2 * (1/WheelRadius)  * Gear;
    w3 = w3 * (1/WheelRadius)  * Gear;

    //to rpm
    w1 = w1 * rad2rpm;
    w2 = w2 * rad2rpm;
    w3 = w3 * rad2rpm;

#ifdef DEBUG
    cout <<  "w1 = " << w1 << endl;
    cout <<  "w2 = " << w2 << endl;
    cout <<  "w3 = " << w3 << endl;
#endif


    //to rpmRatio ( = pwmRatio)
    pwmRatio_1 = ( abs(w1) - (float)RPM_Offset) / (float)RPM_Max;
    pwmRatio_2 = ( abs(w2) - (float)RPM_Offset) / (float)RPM_Max;
    pwmRatio_3 = ( abs(w3) - (float)RPM_Offset) / (float)RPM_Max;


    if(pwmRatio_1 > 1.0 || pwmRatio_2 > 1.0 || pwmRatio_3 > 1.0)
    {
        double pwmMax = pwmRatio_1;
        pwmMax = pwmMax > pwmRatio_2 ? pwmMax : pwmRatio_2;
        pwmMax = pwmMax > pwmRatio_3 ? pwmMax : pwmRatio_3;
        double tRatio = 1.0/(double)pwmMax;
        pwmRatio_1 = pwmRatio_1 * tRatio;
        pwmRatio_2 = pwmRatio_2 * tRatio;
        pwmRatio_3 = pwmRatio_3 * tRatio;
    }
    if(pwmRatio_1 < 0.0 || pwmRatio_2 < 0.0 || pwmRatio_3 < 0.0)
    {
    	q <<"<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
    	q <<"<motor type=\"RunMotor\">\n";
    	q <<"    <motor_1 enable=\"" << 0 << "\" dir=\"" << 0 << "\" speed=\""<< 0 <<"\" />\n";
    	q <<"    <motor_2 enable=\"" << 0 << "\" dir=\"" << 0 << "\" speed=\""<< 0 <<"\" />\n";
    	q <<"    <motor_3 enable=\"" << 0 << "\" dir=\"" << 0 << "\" speed=\""<< 0 <<"\" />\n";
    	q <<"</motor>";		
    	string sendStr = q.str();
    	cout<< sendStr<<endl;
    	return sendStr;
    }


#ifdef DEBUG
    cout <<  "pwmRatio_1 = " << pwmRatio_1 << endl;
    cout <<  "pwmRatio_2 = " << pwmRatio_2 << endl;
    cout <<  "pwmRatio_3 = " << pwmRatio_3 << endl;


    double p1,p2,p3;

    p1 = (int)(  (pwmRatio_1 *percentRange + PWM_Limit_Percent_Min )*PWM_Range   );
    p2 = (int)(  (pwmRatio_2 *percentRange + PWM_Limit_Percent_Min )*PWM_Range   );
    p3 = (int)(  (pwmRatio_3 *percentRange + PWM_Limit_Percent_Min )*PWM_Range   );

    printf("pwm_1 = %lf,pwm_2 = %lf, pwm_3 = %lf\n",p1,p2,p3);

#endif


    pwm_1 = (int)(  (pwmRatio_1 *percentRange + PWM_Limit_Percent_Min )*PWM_Range   );
    pwm_2 = (int)(  (pwmRatio_2 *percentRange + PWM_Limit_Percent_Min )*PWM_Range   );
    pwm_3 = (int)(  (pwmRatio_3 *percentRange + PWM_Limit_Percent_Min )*PWM_Range   );


//#ifdef DEBUG
    /*
    if( (old_pwm_1 != pwm_1) || (old_pwm_2 != pwm_2) ||  (old_pwm_3 != pwm_3)){
        printf("pwm_1 = %d,pwm_2 = %d, pwm_3 = %d(PWM)\n",pwm_1,pwm_2,pwm_3);
        fflush(stdout);

        old_pwm_1 = pwm_1;
        old_pwm_2 = pwm_2;
        old_pwm_3= pwm_3;
    }*/


//#endif


    //====for xml file====
    

    en_1 = en_2 = en_3 = en;

    if(pwm_1 <= PWM_MIN || pwm_1 >= PWM_MAX)    en_1 = 0;
    if(pwm_2 <= PWM_MIN || pwm_2 >= PWM_MAX)    en_2 = 0;
    if(pwm_3 <= PWM_MIN || pwm_3 >= PWM_MAX)    en_3 = 0;

    dir_1 = w1 < 0.0 ? 1 : 0;
    dir_2 = w2 < 0.0 ? 1 : 0;
    dir_3 = w3 < 0.0 ? 1 : 0;


     ROS_INFO("[1:en=%d,dir=%d,pwm=%d] [2:%d,%d,%d] [3:%d,%d,%d]\n",
            en_1,dir_1,pwm_1,
            en_2,dir_2,pwm_2,
            en_3,dir_3,pwm_3);


    
    q <<"<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
    q <<"<motor>\n";
    //q <<"<motor type=\"RunMotor\">\n";
    q <<"    <motor_1 enable=\"" << en_1 << "\" dir=\"" << dir_1 << "\" speed=\""<< pwm_1 <<"\" />\n";
    q <<"    <motor_2 enable=\"" << en_2 << "\" dir=\"" << dir_2 << "\" speed=\""<< pwm_2 <<"\" />\n";
    q <<"    <motor_3 enable=\"" << en_3 << "\" dir=\"" << dir_3 << "\" speed=\""<< pwm_3 <<"\" />\n";
    q <<"</motor>";
    //q <<"<motor type=\"GetEncoder\" />\n";
    string sendStr = q.str();


#ifdef DEBUG
    cout<< sendStr<<endl;
#endif
    return sendStr;

}

string speed2pwm_range(float range_x,float range_y,float range_yaw,int en){
    float v_x,v_y,v_yaw;
    v_x = range_x/(float)SpeedMode_Range_Size * SpeedMode_Range_MaxSpeed;
    v_y = range_y/(float)SpeedMode_Range_Size * SpeedMode_Range_MaxSpeed;
    v_yaw = range_yaw/(float)SpeedMode_Range_Size * SpeedMode_Range_MaxRotation;

    return speed2pwm(v_x,v_y,v_yaw,en);
}

void send2FPGA_ori(string sendStr){
    printf("in sedn=========================\n");
    std::cout << "sendStr   =   " << sendStr <<std::endl;

    LinuxSocket client2;
    client2.setTimeout(5);
    client2.create();

    if( client2.connect ( serverIP , serverPort ) )
    {
        client2.send(sendStr);
    }
    client2.close();

}

void send2FPGA(string sendStr){
    sendLock = true;
    //if(client->send(sendStr)){
    if(client->send((void*)sendStr.c_str(),sendStr.size())){
        string recvStr;
        client->recv(recvStr);
        //cout << "FPGA ECHO : " << recvStr << endl;


#ifdef DEBUG
        cout << sendStr << endl;
#endif
    }

    sendLock = false;
}

double preX = 0.0;
double loseCount = 0.0;
double lossRate = 0.0;

void moveCallback(const geometry_msgs::TwistPtr& msg)
{


#ifdef DEBUG
    //eTime("previous callback end -> now callback");
   // sTime();
#endif

    if(sendLock)    return;


    float x , y , r;//x , y , rotation
    int en,speedType;

    //ROS_INFO("I heard: x = %lf,y=%lf,r=%lf,en=%d,speeedType=%d",
    //        msg->linear.x, msg->linear.y,r = msg->angular.z, msg->angular.x, msg->angular.y);

    x = msg->linear.x;
    y = msg->linear.y;
    r = msg->angular.z;
    //note!!very triky
    //note!!very triky
    en = msg->angular.x;
    speedType = msg->angular.y;

    if(speedType==0)  speedType = SpeedMode_MS;

    x = abs(x) < 0.01 ? 0.0 : x;
    y = abs(y) < 0.01 ? 0.0 : y;
    r = abs(r) < 0.001 ? 0.0 : r;

    //ROS_INFO("After I heard: x = %lf,y=%lf,r=%lf,en=%d,speeedType=%d", x,y,r,en,speedType);

    string sendStr;
    if(speedType==SpeedMode_MS){
        sendStr = speed2pwm(x,y,r,en);
    }else {
        sendStr = speed2pwm_range(x,y,r,en);
    }

    //sTime();
    send2FPGA_ori(sendStr);
    //send2FPGA(sendStr);
    //eTime("send FPGA Time");

#ifdef DEBUG
    double ms = eTime("motion using Time (include sendFPGA)");
    averageTime(ms);

    //sTime();
    //ROS_INFO("END Time"   );
#endif
}



bool initConnectFPGA(){
    client = new LinuxSocket();
    client->setTimeout(5);
    client->create();

    if( client->connect ( serverIP , serverPort ) )
    {
        //cout << "System::motion say [Connect FPGA Success]" << endl;
        return true;
    }else{
        cout << "System::motion say [Connect FPGA 'FAIL']" << endl;
        return false;
    }
}


int main(int argc, char **argv){

//    if(!initConnectFPGA()){
//        return -1;
//    }

    int robotIndex = Utility::parseArg_getRobotIndex(argc,argv);
    string robotSpeed_topicName = "";
    if(robotIndex==1)           robotSpeed_topicName = R1_Strategy_PathPlan_RobotSpeed_Topic;
    else if(robotIndex==2)      robotSpeed_topicName = R2_Strategy_PathPlan_RobotSpeed_Topic;
    else if(robotIndex==3)      robotSpeed_topicName = R3_Strategy_PathPlan_RobotSpeed_Topic;

    ros::init(argc, argv, Node_Name);
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe(robotSpeed_topicName, 10, moveCallback);
    ros::spin();

    client->close();

    return 0;
}
