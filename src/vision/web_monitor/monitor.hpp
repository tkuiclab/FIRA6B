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
  int dis_max;        //pix
  int dis_min;        //pix
  int ang_max;        //pix
  int ang_min;        //pix
  int x;              //pix
  int y;              //pix
  int angle;          //pix
  double distance;    //pix
  int size;

  int left_dis;       //pix
  int right_dis;      //pix
  int left_x;         //pix
  int left_y;         //pix
  int right_x;        //pix
  int right_y;        //pix
  int fix_x;          //pix
  int fix_y;          //pix
  int fix_angle;      //pix
  int fix_distance;   //pix
  int fix_ang_max;    //pix
  int fix_ang_min;    //pix
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

  int b_end_gap; 
  int y_end_gap;
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

  //vector<BYTE> ColorFile();
  void imageCb(const sensor_msgs::ImageConstPtr&);
  void Parameter_getting(const int x);
  void SaveButton_setting(const vision::bin msg);
  void View(const vision::view msg);
  int mosue_x,mosue_y;

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
///////////////////////FPS/////////////////////////////////
  double dt;
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

  void objectdet_change(Mat &, int, object_Item &);
  void Mark_point(Mat &, int, int, int, int, int &, int);
  void object_Item_reset(object_Item &);
  void find_around(Mat &, int, int, int &, int);
  void draw_ellipse(Mat &, object_Item &,int );
  void draw_Line(Mat &, int, int, int);
  void Draw_cross(cv::Mat &,char);
  void find_object_point(object_Item &, int);
  void HSVmap();

  int Angle_Interval(int);
  int Strategy_Angle(int angle);
  double camera_f(double Omni_pixel);
  double Omni_distance(double dis_pixel);
};
