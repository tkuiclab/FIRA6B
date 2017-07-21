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
#include "vision/Two_point.h"
#include "vision/Object.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sstream>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <ros/package.h>
#include "vision/bin.h"
#include "vision/view.h"
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
  ros::Publisher Two_point_pub;
  ros::Subscriber s1;
  ros::Subscriber s2;

  cv::Mat *frame;
  cv::Mat *outputframe;
  cv::Mat Main_frame;
  cv::Mat Findmap;
  cv::Mat FIRA_map;
  cv::Mat Obstaclemap;
  cv::Mat Erodemap;
  cv::Mat Dilatemap;

 int hmax,hmin,smax,smin,vmax,vmin;
  object_Item FIND_Item,Red_Item,Yellow_Item,Blue_Item;
  object_Item *Obstacle_Item;
  int dont_angle[6];
  int frame_counter;
  vector<double> Angle_sin;
  vector<double> Angle_cos;
  vector<BYTE> color_map;
  deque<int> find_point;

  int search_angle;
  int search_distance;
  int search_start;
  int search_near;
  int search_middle;
  int search_end;
  int Camera_H;
  double Camera_f;
  int center_x, center_y, center_inner, center_outer, center_front;
  long int EndTime;
  std::string vision_path;

  int dis_gap;
  int white_gray,white_angle;

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
  int BlackAngleMsg;
  int WhiteGrayMsg;
  int WhiteAngleMsg;
  int fpsMsg;
  int BallHSVBoxMsg[6];
  int GreenHSVBoxMsg[6];
  int BlueHSVBoxMsg[6];
  int YellowHSVBoxMsg[6];
  int WhiteHSVBoxMsg[6];
  int ColorModeMsg;
  int paraMeterCheck;
  int SaveButton;
  int viewcheck;
  int image_fps;
  int   ball_x, ball_y, ball_ang, ball_dis;
  int   blue_x, blue_y,blue_ang, blue_dis;
  int   yellow_x, yellow_y, yellow_ang, yellow_dis;
  std::string  ball_LR,blue_LR,yellow_LR;
  //vector<BYTE> ColorFile();
  void imageCb(const sensor_msgs::ImageConstPtr&);
  void Parameter_getting(const int x);
  void SaveButton_setting(const vision::bin msg);
  void View(const vision::view msg);
  int mosue_x,mosue_y;
  int distance_space[100];
  int distance_pixel[100];
/////////////////init_data///////////////////////////////////////////////////
  void init_data(){
    ball_LR = "NULL";blue_LR = "NULL";yellow_LR = "NULL";
    image_fps = 999;
    ball_x = 999; ball_y = 999; ball_ang = 999; ball_dis = 999;
    blue_x = 999; blue_y = 999; blue_ang = 999; blue_dis = 999;
    yellow_x = 999; yellow_y = 999; yellow_ang = 999; yellow_dis = 999;
  }
/////////////////////////////////////////////////////////////////////
  void object_compare(int distance ,int angle){
    if(FIND_Item.dis_max < distance){
      FIND_Item.dis_max = distance;
    }
    if(FIND_Item.dis_min > distance){
      FIND_Item.dis_min = distance;
    }

    if(FIND_Item.ang_max < angle){
      FIND_Item.ang_max = angle;
    }
    if(FIND_Item.ang_min > angle){
      FIND_Item.ang_min = angle;
    }
  }
/////////////////////////////////////////////////////////////
//////////////////////////色彩空間////////////////////////////
  vector<BYTE> ColorFile()
  {
    string Filename =vision_path + FILE_PATH;
    const char *Filename_Path = Filename.c_str();
    // open the file:
    streampos fileSize;
    std::ifstream file(Filename_Path, ios::binary);
    // get its size:
    file.seekg(0, ios::end);
    fileSize = file.tellg();
    file.seekg(0, ios::beg);
    // read the data:
    vector<BYTE> fileData(fileSize);
    file.read((char*) &fileData[0], fileSize);
    return fileData;
  }
///////////////////////////////////////////////////////////
///////////////////////FPS/////////////////////////////////
  //int frame_counter=0;
  int topic_counter=0;
  // long int EndTime;
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

//////////////////////SCAN/////////////////////
  std::vector<int>scan_para;
  std::vector<int>scan_near;
  std::vector<int>scan_middle;
  std::vector<int>scan_far;

  std::vector<int>dis_space;
  std::vector<int>dis_pixel;
/////////////////////HSV///////////////////////
  int HSV_Ball[6];
  int HSV_Yellow[6];
  int HSV_Green[6];
  int HSV_Blue[6];
  vector<int> HSV_red;
  vector<int> HSV_green;
  vector<int> HSV_blue;
  vector<int> HSV_yellow;
  vector<int> HSV_white;


  //int angle_for_distance(int dis);
  void objectdet_change(Mat &, int, object_Item &);
  void creat_Obstclemap(Mat &, int);
  void creat_FIRA_map(Mat &, Mat &);

  void objectdet_Obstacle(Mat &, int, object_Item *);
  void Mark_point(Mat &, int, int, int, int, int &, int);
  void object_Item_reset(object_Item &);
  void find_around(Mat &, int, int, int &, int);
  //void object_compare(int, int);
  void draw_ellipse(Mat &, object_Item &,int );
  void draw_Line(Mat &, int, int, int);
  void Draw_cross(cv::Mat &,char);
  void find_object_point(object_Item &, int);

  void HSVmap();
  int Angle_Interval(int);

  double camera_f(double Omni_pixel);
  double Omni_distance(double dis_pixel);

};
