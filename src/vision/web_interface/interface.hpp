#include <cstdio>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include <vector>
#include <deque>
#include <iostream>
#include <fstream>
#include <vision/parameterbutton.h>
#include "vision/parametercheck.h"
#include "vision/center.h"
#include "vision/white.h"
#include "vision/camera.h"
#include "vision/black.h"
#include "vision/colorbutton.h"
#include "vision/scan.h"
#include "vision/color.h"
#include "vision/Object.h"
#include "vision/dis.h"
#include "vision/position.h"
#include "vision/Two_point.h"
#include "vision/bin.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sstream>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <ros/package.h>
#include "std_msgs/Int32MultiArray.h"

#define FILE_PATH "/config/HSVcolormap.bin"
using namespace cv;
using namespace std;
typedef unsigned char BYTE;
namespace enc = sensor_msgs::image_encodings;

class object_Item{
  public:
  int dis_max;
  int dis_min;
  int ang_max;
  int ang_min;
  int x;
  int y;
  int angle;
  double distance;
  int size;
  string LR;
};

class InterfaceProc
{
private:
  ros::NodeHandle nh;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_threshold_;
  ros::Publisher object_pub;
  ros::Publisher CenterDis_pub;
  ros::Publisher white_pub;
  ros::Publisher black_pub;
  ros::Publisher Two_point_pub;

  ros::Subscriber s1;
  ros::Subscriber s2;
  ros::Subscriber s3;
  ros::Subscriber s4;
  ros::Subscriber s5;
  ros::Subscriber s6;
  ros::Subscriber s7;
  ros::Subscriber s8;
  ros::Subscriber s9;
  ros::Subscriber s10;
  ros::Subscriber s11;
  cv::Mat *frame_white;
  cv::Mat *frame_black;
  cv::Mat *frame;
  cv::Mat *centermats;

  cv::Mat Main_frame;
  cv::Mat Findmap;
  cv::Mat FIRA_map;
  cv::Mat Obstaclemap;
  cv::Mat Erodemap;
  cv::Mat Dilatemap;

  cv::Mat *CenterModels;
  cv::Mat *ColorModels;
  cv::Mat *CameraModels;
  cv::Mat *WhiteModels;
  cv::Mat *BlackModels;
  cv::Mat *ScanModels;
  cv::Mat *outputframe;

  int hmax,hmin,smax,smin,vmax,vmin;
  int Unscaned_Angle[8];

  vector<double> Angle_sin;
  vector<double> Angle_cos;

  int search_angle;
  int search_distance;
  int search_start;
  int search_near;
  int search_middle;
  int search_end;
  int Camera_H;
  double Camera_f;
  int center_x, center_y, center_inner, center_outer, center_front;

  std::string vision_path;

public:
  InterfaceProc();
  ~InterfaceProc();
  int buttonmsg;
  int CenterXMsg;
  int CenterYMsg;
  int robotCenterX;
  int robotCenterY;
  int InnerMsg;
  int OuterMsg;
  int FrontMsg;
  double Camera_HighMsg;
  int colorbottonMsg;
  int Angle_Near_GapMsg;
  int Magn_Near_GapMsg;
  int Magn_Near_StartMsg;
  int Magn_Middle_StartMsg;
  int Magn_Far_StartMsg;
  int Magn_Far_EndMsg;
  int Dont_Search_Angle_1Msg;
  int Dont_Search_Angle_2Msg;
  int Dont_Search_Angle_3Msg;
  int Angle_range_1Msg;
  int Angle_range_2_3Msg;
  int BlackGrayMsg;
  double BlackAngleMsg;
  int WhiteGrayMsg;
  double WhiteAngleMsg;
  int fpsMsg;
  int BallHSVBoxMsg[6];
  int GreenHSVBoxMsg[6];
  int BlueHSVBoxMsg[6];
  int YellowHSVBoxMsg[6];
  int WhiteHSVBoxMsg[6];
  int ColorModeMsg;
  int paraMeterCheck;
  int SaveButton;
 
  void imageCb(const sensor_msgs::ImageConstPtr&);
  void ParameterButtonCall(const vision::parameterbutton);
  void colorcall(const vision::color);
  void centercall(const vision::center);
  void whitecall(const vision::white);
  void cameracall(const vision::camera);
  void blackcall(const vision::black);
  void colorbuttoncall(const vision::colorbutton);
  void scancall(const vision::scan);
  void Parameter_getting(const int x) ;
  void Parameter_setting(const vision::parametercheck) ;
  void positioncall(const vision::position msg);
  void SaveButton_setting(const vision::bin );

  int mosue_x,mosue_y;
///////////////////////FPS/////////////////////////////////
  double dt;
  double Exposure_mm;
  void set_campara(int value_ex){
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;
    double exposure = (double)value_ex/1000;
    double_param.name = "exposure";
    double_param.value = exposure;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;
    ros::service::call("/prosilica_driver/set_parameters", srv_req, srv_resp);
  }
  double camera_exposure;
  void get_campara(){
    camera_exposure = 0.025;
    nh.getParam("/prosilica_driver/exposure",camera_exposure);
  }
/////////////////////HSV///////////////////////
  vector<int> HSV_red;
  vector<int> HSV_green;
  vector<int> HSV_blue;
  vector<int> HSV_yellow;
  vector<int> HSV_white;

  int Angle_Interval(int);

  double camera_f(double Omni_pixel);
  double Omni_distance(double dis_pixel);

  cv::Mat ColorModel(const cv::Mat iframe);
  cv::Mat WhiteModel(const cv::Mat iframe);
  cv::Mat CenterModel(const cv::Mat iframe);
  cv::Mat ScanModel(const cv::Mat iframe);
  cv::Mat CameraModel(const cv::Mat iframe);
  cv::Mat White_Line(const cv::Mat iframe);
  cv::Mat Black_Line(const cv::Mat iframe);
};
