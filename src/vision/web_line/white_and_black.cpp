#include "white_and_black.hpp"
#include "math.h"
#define PI 3.14159265
#define FRAME_COLS 659 //width  x695
#define FRAME_ROWS 493 //height y493
using namespace std;
using namespace cv;

//std::string visionpath = ros::package::getPath("vision");
std::string visionpath = ros::package::getPath("fira_launch");

std::string parameterpath = "/default_config/vision_better.yaml";
//std::string parameterpath = "/config/Parameter.yaml";
std::string param = visionpath + parameterpath; 
const char *parampath = param.c_str();

void InterfaceProc::Parameter_getting(const int x)
{
    nh.getParam("/FIRA/HSV/white/gray",WhiteGrayMsg);
    nh.getParam("/FIRA/HSV/white/angle",WhiteAngleMsg);
    nh.getParam("/FIRA/HSV/black/gray",BlackGrayMsg);
    nh.getParam("/FIRA/HSV/black/angle",BlackAngleMsg);
    nh.getParam("/FIRA/Center/Center_X",CenterXMsg);
    nh.getParam("/FIRA/Center/Center_Y",CenterYMsg);
    nh.getParam("/FIRA/Center/Inner",InnerMsg);
    nh.getParam("/FIRA/Center/Outer",OuterMsg);
    nh.getParam("/FIRA/Center/Camera_high",Camera_HighMsg);
  cout<<WhiteAngleMsg<<endl;
}
int Frame_area(int num,int range){
  if(num < 0) num = 0;
  else if(num >= range) num = range-1;
  return num;
}
InterfaceProc::InterfaceProc()
   :it_(nh)
{
    ros::NodeHandle n("~");
    //Parameter_getting(1);	
    image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &InterfaceProc::imageCb, this);
    //image_sub_ = it_.subscribe("/camera/image_raw", 1, &InterfaceProc::imageCb, this);
    white_pub  = nh.advertise<std_msgs::Int32MultiArray>("/vision/whiteRealDis",1);
    black_pub  = nh.advertise<std_msgs::Int32MultiArray>("/vision/blackRealDis",1);
    frame=new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS),CV_8UC3 );
    nh.getParam("/FIRA/HSV/white/gray",WhiteGrayMsg);
    nh.getParam("/FIRA/HSV/white/angle",WhiteAngleMsg);
    nh.getParam("/FIRA/HSV/black/gray",BlackGrayMsg);
    nh.getParam("/FIRA/HSV/black/angle",BlackAngleMsg);
    nh.getParam("/FIRA/Center/Center_X",CenterXMsg);
    nh.getParam("/FIRA/Center/Center_Y",CenterYMsg);
    nh.getParam("/FIRA/Center/Inner",InnerMsg);
    nh.getParam("/FIRA/Center/Outer",OuterMsg);
    nh.getParam("/FIRA/Center/Camera_high",Camera_HighMsg);
} 
InterfaceProc::~InterfaceProc()
{
 delete frame;

}
void InterfaceProc::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    int StartTime = ros::Time::now().toNSec();
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Mat frame;
    cv::flip(cv_ptr->image, frame, 1);
    /*nh.getParam("/FIRA/HSV/white/gray",WhiteGrayMsg);
    nh.getParam("/FIRA/HSV/white/angle",WhiteAngleMsg);
    nh.getParam("/FIRA/HSV/black/gray",BlackGrayMsg);
    nh.getParam("/FIRA/HSV/black/angle",BlackAngleMsg);
    nh.getParam("/FIRA/Center/Center_X",CenterXMsg);
    nh.getParam("/FIRA/Center/Center_Y",CenterYMsg);
    nh.getParam("/FIRA/Center/Inner",InnerMsg);
    nh.getParam("/FIRA/Center/Outer",OuterMsg);
    nh.getParam("/FIRA/Center/Camera_high",Camera_HighMsg);*/
    double ang_PI;
    for(int ang=0 ; ang<360; ang++){
     ang_PI = ang*PI/180;
     Angle_sin.push_back(sin(ang_PI));
     Angle_cos.push_back(cos(ang_PI));
  	}
///////////////////////////////White_Line///////////////////////////////
  cv::Mat whiteframe(cv::Size(frame.cols,frame.rows), CV_8UC3);
  whiteframe=frame;
  int white_dis;
  for(int i=0;i<whiteframe.rows;i++){
    for(int j=0;j<whiteframe.cols;j++){
      unsigned char gray = ( whiteframe.data[(i*whiteframe.cols*3)+(j*3)+0]
                           + whiteframe.data[(i*whiteframe.cols*3)+(j*3)+1]
                           + whiteframe.data[(i*whiteframe.cols*3)+(j*3)+2])/3;
      if(gray < WhiteGrayMsg){
        whiteframe.data[(i*whiteframe.cols*3)+(j*3)+0] = 0;
        whiteframe.data[(i*whiteframe.cols*3)+(j*3)+1] = 0;
        whiteframe.data[(i*whiteframe.cols*3)+(j*3)+2] = 0;
      }else{
        whiteframe.data[(i*whiteframe.cols*3)+(j*3)+0] = 255;
        whiteframe.data[(i*whiteframe.cols*3)+(j*3)+1] = 255;
        whiteframe.data[(i*whiteframe.cols*3)+(j*3)+2] = 255;
      }
    }

    whiteItem_pixel.clear();
    WhiteRealDis.data.clear();
    Parameter_getting(1);
 for(double angle = 0; angle < 360; angle = angle + WhiteAngleMsg){
          double angle_be = angle+FrontMsg;

          if(angle_be >= 360) angle_be -= 360;

          double x_ = cos((angle_be*PI)/180);//Angle_cos[angle_be];
          double y_ = sin((angle_be*PI)/180);//Angle_sin[angle_be];

          for(int r = InnerMsg; r <= OuterMsg; r++){
              int dis_x = x_*r;
              int dis_y = y_*r;

              int image_x = Frame_area(center_x+dis_x,whiteframe.cols);
              int image_y = Frame_area(center_y-dis_y,whiteframe.rows);

              if( whiteframe.data[(image_y*whiteframe.cols + image_x)*3+0] == 255
                &&whiteframe.data[(image_y*whiteframe.cols + image_x)*3+1] == 255
                &&whiteframe.data[(image_y*whiteframe.cols + image_x)*3+2] == 255){
                  whiteItem_pixel.push_back(hypot(dis_x,dis_y));
                  break;
              }else{
                  whiteframe.data[(image_y*whiteframe.cols + image_x)*3+0] = 0;
                  whiteframe.data[(image_y*whiteframe.cols + image_x)*3+1] = 0;
                  whiteframe.data[(image_y*whiteframe.cols + image_x)*3+2] = 255;
              }
              if(r==OuterMsg){
                whiteItem_pixel.push_back(999);
              }
          }
      }
      for(int j=0;j<whiteItem_pixel.size();j++){
        white_dis = Omni_distance(whiteItem_pixel[j]);
        WhiteRealDis.data.push_back(white_dis);
      }
      white_pub.publish(WhiteRealDis);
///////////////////////////////////////////////////////////////////////b
///////////////////////////////BlackItem////////////////////////////////
  cv::Mat blackframe(cv::Size(frame.cols,frame.rows), CV_8UC3);
  blackframe = frame;
  int black_dis;
  for(int i=0;i<blackframe.rows;i++){
    for(int j=0;j<blackframe.cols;j++){
      unsigned char gray = ( blackframe.data[(i*blackframe.cols*3)+(j*3)+0]
                           + blackframe.data[(i*blackframe.cols*3)+(j*3)+1]
                           + blackframe.data[(i*blackframe.cols*3)+(j*3)+2])/3;
      if(gray < BlackGrayMsg){
        blackframe.data[(i*blackframe.cols*3)+(j*3)+0] = 0;
        blackframe.data[(i*blackframe.cols*3)+(j*3)+1] = 0;
        blackframe.data[(i*blackframe.cols*3)+(j*3)+2] = 0;
      }else{
        blackframe.data[(i*blackframe.cols*3)+(j*3)+0] = 255;
        blackframe.data[(i*blackframe.cols*3)+(j*3)+1] = 255;
        blackframe.data[(i*blackframe.cols*3)+(j*3)+2] = 255;
      }
    }
  }
      blackItem_pixel.clear();
      BlackRealDis.data.clear();
      Parameter_getting(1);
      for(double angle = 0; angle < 360; angle = angle + BlackAngleMsg){
          double angle_be = angle+FrontMsg;

          if(angle_be >= 360) angle_be -= 360;

          double x_ = cos((angle_be*PI)/180);//Angle_cos[angle_be];
          double y_ = sin((angle_be*PI)/180);//Angle_sin[angle_be];
          for(int r = InnerMsg; r <= OuterMsg; r++){
              int dis_x = x_*r;
              int dis_y = y_*r;

              int image_x = Frame_area(center_x+dis_x,blackframe.cols);
              int image_y = Frame_area(center_y-dis_y,blackframe.rows);

              if( blackframe.data[(image_y*blackframe.cols + image_x)*3+0] == 0
                &&blackframe.data[(image_y*blackframe.cols + image_x)*3+1] == 0
                &&blackframe.data[(image_y*blackframe.cols + image_x)*3+2] == 0){
                  blackItem_pixel.push_back(hypot(dis_x,dis_y));
                  break;
              }else{
                  blackframe.data[(image_y*blackframe.cols + image_x)*3+0] = 0;
                  blackframe.data[(image_y*blackframe.cols + image_x)*3+1] = 0;
                  blackframe.data[(image_y*blackframe.cols + image_x)*3+2] = 255;
              }
              if(r==OuterMsg){
                blackItem_pixel.push_back(999);
              }
          }
      }

      for(int j=0;j<blackItem_pixel.size();j++){
        black_dis = Omni_distance(blackItem_pixel[j]);
        BlackRealDis.data.push_back(black_dis);
      }
      black_pub.publish(BlackRealDis);
    }
      //////////////////////////////////////////////////////////
}
double InterfaceProc::Omni_distance(double pixel_dis)
{
  double Z = -1*Camera_HighMsg;  //Camera_HighMsg=65;
  //double c  =  D0/2;
  double c = 83.125; 
  double b = c*0.8722;
  double f = camera_f(OuterMsg*2*0.9784);
  double r = atan2(f,pixel_dis*0.0099);
  double dis = Z*(pow(b,2)-pow(c,2))*cos(r) / ((pow(b,2)+pow(c,2))*sin(r) -2*b*c)*0.1;
  if(dis < 0 || dis > 999){dis = 999;}
  //ROS_INFO("%f %f %f %f",Z,c,r,dis);
  return dis;
}
double InterfaceProc::camera_f(double Omni_pixel)
{
  double m = (Omni_pixel*0.0099)/60;        // m = H1/H0 = D1/D0    D0 + D1 = 180
  double D0 = 180/(1+m);                    // D1 = m   *D0
  double D1 = 180/(1+(1/m));                // D0 = 1/m *D1
  double f = 1/(1/D0 + 1/D1);
  //ROS_INFO("m = %f D0 = %f D1 = %f F = %f",m,D0,D1,f);
  return D1;
}
