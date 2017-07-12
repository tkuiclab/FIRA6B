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
#include "vision/parametercheck.h"
#include "vision/white.h"
#include "vision/black.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sstream>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <ros/package.h>
#include "std_msgs/Int32MultiArray.h"
#include "vision/bin.h"

using namespace cv;
using namespace std;
typedef unsigned char BYTE;
namespace enc = sensor_msgs::image_encodings;

class InterfaceProc
{
private:
  ros::NodeHandle nh;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher white_pub;
  ros::Publisher black_pub;
  ros::Subscriber s1;
  cv::Mat *frame;


  std_msgs::Int32MultiArray WhiteRealDis;
  std_msgs::Int32MultiArray BlackRealDis;

 // deque<double> WhiteDis;
  //deque<double> BlackDis;

  double WhiteDis;
  double BlackDis;
  vector<double> Angle_sin;
  vector<double> Angle_cos;
  vector<int>whiteItem_pixel;
  vector<int>blackItem_pixel;
public:
  InterfaceProc();
  ~InterfaceProc();

  int BlackGrayMsg;
  double BlackAngleMsg;
  int WhiteGrayMsg;
  double WhiteAngleMsg;
  int InnerMsg;
  int OuterMsg;
  int CenterXMsg;
  int CenterYMsg;
  int FrontMsg;
  double Camera_HighMsg;
  int Camera_H;
  double Camera_f;
  int center_x, center_y, center_inner, center_outer, center_front;
  void imageCb(const sensor_msgs::ImageConstPtr&);
  void Parameter_getting(const int x) ;
  void SaveButton_setting(const vision::bin msg);
  double Omni_distance(double dis_pixel);
  double camera_f(double Omni_pixel);
/////////////////////////////////////////////////////////////

};
