#define PI 3.14159265
#include "interface.hpp"
#include "math.h"
#include <time.h>
#define FRAME_COLS 659 //width  x695
#define FRAME_ROWS 493 //height y493
#define REDITEM 0x01
#define GREENITEM 0x02
#define BLUEITEM 0x04
#define YELLOWITEM 0x08
#define WHITEITEM 0x10
#define OBSTACLEITEM 0x00
#define IMAGE_TEST1 "/home/testa/image_transport_ws/src/interface_ws/vision/1.jpg"//圖片路徑
static const std::string OPENCV_WINDOW = "Image window";
using namespace std;
namespace enc = sensor_msgs::image_encodings;
const double ALPHA = 0.5;

//std::string visionpath = ros::package::getPath("vision");
std::string visionpath = ros::package::getPath("fira_launch");

std::string parameterpath = "/default_config/vision_better.yaml";
//std::string parameterpath = "/config/Parameter.yaml";
std::string param = visionpath + parameterpath;
const char *parampath = param.c_str();

void onMouse(int Event, int x, int y, int flags, void* param);
int mousex = -1 , mousey = -1 , onclick = 0;
void InterfaceProc::ParameterButtonCall (const vision::parameterbutton msg)
{
  buttonmsg = msg.button;
  std::cout << buttonmsg << std::endl;
}
void InterfaceProc::colorcall(const vision::color msg)
{
  ColorModeMsg = msg.ColorMode;
  switch (ColorModeMsg)
  {
  case 0:
    for (int i = 0; i < 6; i++)BallHSVBoxMsg[i] = msg.BallHSVBox[i];
    break;
  case 1:
    for (int i = 0; i < 6; i++)GreenHSVBoxMsg[i] = msg.GreenHSVBox[i];
    break;
  case 2:
    for (int i = 0; i < 6; i++)BlueHSVBoxMsg[i] = msg.BlueHSVBox[i];
    break;
  case 3:
    for (int i = 0; i < 6; i++)YellowHSVBoxMsg[i] = msg.YellowHSVBox[i];
    break;
  case 4:
    for (int i = 0; i < 6; i++)WhiteHSVBoxMsg[i] = msg.WhiteHSVBox[i];
    break;
  }
}
void InterfaceProc::centercall(const vision::center msg)
{
  CenterXMsg = msg.CenterX;
  CenterYMsg = msg.CenterY;
  InnerMsg = msg.Inner;
  OuterMsg = msg.Outer;
  FrontMsg = msg.Front;
  Camera_HighMsg = msg.Camera_High;

  center_x = msg.CenterX;
  center_y = msg.CenterY;
  center_inner = msg.Inner;
  center_outer = msg.Outer;
  center_front = msg.Front;

}
void InterfaceProc::whitecall(const vision::white msg)
{
  WhiteGrayMsg = msg.Gray;
  WhiteAngleMsg = msg.Angle;
}
void InterfaceProc::cameracall(const vision::camera msg)
{
  fpsMsg = msg.fps;
}
void InterfaceProc::blackcall(const vision::black msg)
{
  BlackGrayMsg = msg.Gray;
  BlackAngleMsg = msg.Angle;
}
void InterfaceProc::colorbuttoncall(const vision::colorbutton msg)
{
  colorbottonMsg = msg.button;
}
void InterfaceProc::scancall(const vision::scan msg)
{
  Angle_Near_GapMsg = msg.Angle_Near_Gap;
  Magn_Near_GapMsg = msg.Magn_Near_Gap;
  Magn_Near_StartMsg = msg.Magn_Near_Start;
  Magn_Middle_StartMsg = msg.Magn_Middle_Start;
  Magn_Far_StartMsg = msg.Magn_Far_Start;
  Magn_Far_EndMsg = msg.Magn_Far_End;
  Dont_Search_Angle_1Msg = msg.Dont_Search_Angle_1;
  Dont_Search_Angle_2Msg = msg.Dont_Search_Angle_2;
  Dont_Search_Angle_3Msg = msg.Dont_Search_Angle_3;
  Angle_range_1Msg = msg.Angle_range_1;
  Angle_range_2_3Msg = msg.Angle_range_2_3;

  search_angle    = msg.Angle_Near_Gap;
  search_distance = msg.Magn_Near_Gap;
  search_start    = msg.Magn_Near_Start;
  search_near     = msg.Magn_Middle_Start;
  search_middle   = msg.Magn_Far_Start;
  search_end      = msg.Magn_Far_End;

}
void InterfaceProc::positioncall(const vision::position msg)
{
  onclick = 1;
  mousex = msg.PositionX;
  mousey = msg.PositionY;
}
void InterfaceProc::Parameter_getting(const int x)
{
  if (ifstream(parampath)) {
    std::string temp = "rosparam load " + param;
    const char *load = temp.c_str();
    system(load);
    cout << "Read the yaml file" << endl;
  }/*else{
    HSV_Ball[0] = 0;  HSV_Ball[1] = 37;
    HSV_Ball[2] = 28; HSV_Ball[3] = 100;
    HSV_Ball[4] = 20; HSV_Ball[5] = 100;

    HSV_Blue[0] = 205;HSV_Blue[1] = 231;
    HSV_Blue[2] = 80; HSV_Blue[3] = 100;
    HSV_Blue[4] = 42; HSV_Blue[5] = 100;

    HSV_Yellow[0] = 39; HSV_Yellow[1] = 10;
    HSV_Yellow[2] = 0; HSV_Yellow[3] = 64;
    HSV_Yellow[4] = 3; HSV_Yellow[5] = 100;

    HSV_Green[0] = 43; HSV_Green[1] = 187;
    HSV_Green[2] = 41; HSV_Green[3] = 74;
    HSV_Green[4] = 21; HSV_Green[5] = 100;
    for(int i=0;i<6;i++){
      HSV_red.push_back(HSV_Ball[i]);  HSV_green.push_back(HSV_Green[i]);
      HSV_blue.push_back(HSV_Blue[i]); HSV_yellow.push_back(HSV_Yellow[i]);
    }
    nh.setParam("/FIRA/HSV/Ball",HSV_red);
    nh.setParam("/FIRA/HSV/Blue",HSV_blue);
    nh.setParam("/FIRA/HSV/Yellow",HSV_yellow);
    nh.setParam("/FIRA/HSV/Green",HSV_green);
    nh.setParam("/FIRA/HSV/White",HSV_white);
    nh.setParam("/FIRA/HSV/ColorMode",1);
    nh.setParam("/FIRA/HSV/white/gray",50);
    nh.setParam("/FIRA/HSV/white/angle",50);
    nh.setParam("/FIRA/HSV/black/gray",50);
    nh.setParam("/FIRA/HSV/black/angle",50);
/////////////////////////////////掃瞄點前置參數///////////////////////////////////
    nh.setParam("/FIRA/SCAN/Angle_Near_Gap",20);
    nh.setParam("/FIRA/SCAN/Magn_Near_Gap",50);
    nh.setParam("/FIRA/SCAN/Magn_Near_Start",120);
    nh.setParam("/FIRA/SCAN/Magn_Middle_Start",120);
    nh.setParam("/FIRA/SCAN/Magn_Far_Start",120);
    nh.setParam("/FIRA/SCAN/Magn_Far_End",120);
    nh.setParam("/FIRA/SCAN/Dont_Search_Angle_1",175);
    nh.setParam("/FIRA/SCAN/Dont_Search_Angle_2",175);
    nh.setParam("/FIRA/SCAN/Dont_Search_Angle_3",175);
    nh.setParam("/FIRA/SCAN/Angle_range_1",45);
  nh.setParam("/FIRA/SCAN/Angle_range_2_3",20);
///////////////////////////////////FPS設定////////////////////////////////////////////////
    nh.setParam("/FIRA/FPS",25);
    get_campara();
//////////////////////////////////CNETER設定///////////////////////////////////////////////
    nh.setParam("/FIRA/Center/Center_X",337);
    nh.setParam("/FIRA/Center/Center_Y",268);
    nh.setParam("/FIRA/Center/Inner",35);
    nh.setParam("/FIRA/Center/Outer",265);
    nh.setParam("/FIRA/Center/Front",146);
    nh.setParam("/FIRA/Center/Camera_high",65);

///////////////////////////////////////////////////////////////////////////////////////////
    nh.setParam("/FIRA/Parameterbutton",1);
    system("rosparam dump"+parampath);
    cout<<"Parameter is created "<<endl;
  }*/
  nh.getParam("/FIRA/HSV/Ball", HSV_red);
  nh.getParam("/FIRA/HSV/Blue", HSV_blue);
  nh.getParam("/FIRA/HSV/Yellow", HSV_yellow);
  nh.getParam("/FIRA/HSV/Green", HSV_green);
  nh.getParam("/FIRA/HSV/White", HSV_white);
  nh.getParam("/FIRA/HSV/ColorMode", ColorModeMsg);
  nh.getParam("/FIRA/HSV/white/gray", WhiteGrayMsg);
  nh.getParam("/FIRA/HSV/white/angle", WhiteAngleMsg);
  nh.getParam("/FIRA/HSV/black/gray", BlackGrayMsg);
  nh.getParam("/FIRA/HSV/black/angle", BlackAngleMsg);
/////////////////////////////////掃瞄點前置參數///////////////////////////////////
  nh.getParam("/FIRA/SCAN/Angle_Near_Gap", Angle_Near_GapMsg);
  nh.getParam("/FIRA/SCAN/Magn_Near_Gap", Magn_Near_GapMsg);
  nh.getParam("/FIRA/SCAN/Magn_Near_Start", Magn_Near_StartMsg);
  nh.getParam("/FIRA/SCAN/Magn_Middle_Start", Magn_Middle_StartMsg);
  nh.getParam("/FIRA/SCAN/Magn_Far_Start", Magn_Far_StartMsg);
  nh.getParam("/FIRA/SCAN/Magn_Far_End", Magn_Far_EndMsg);
  nh.getParam("/FIRA/SCAN/Dont_Search_Angle_1", Dont_Search_Angle_1Msg);
  nh.getParam("/FIRA/SCAN/Dont_Search_Angle_2", Dont_Search_Angle_2Msg);
  nh.getParam("/FIRA/SCAN/Dont_Search_Angle_3", Dont_Search_Angle_3Msg);
  nh.getParam("/FIRA/SCAN/Angle_range_1", Angle_range_1Msg);
  nh.getParam("/FIRA/SCAN/Angle_range_2_3", Angle_range_2_3Msg);

  search_angle    = Angle_Near_GapMsg;
  search_distance = Magn_Near_GapMsg;
  search_start    = Magn_Near_StartMsg;
  search_near     = Magn_Middle_StartMsg;
  search_middle   = Magn_Far_StartMsg;
  search_end      = Magn_Far_EndMsg;


///////////////////////////////////////FPS設定////////////////////////////////////////////////
  nh.getParam("/FIRA/FPS", fpsMsg);
  get_campara();
//////////////////////////////////// CNETER設定///////////////////////////////////////////////
  nh.getParam("/FIRA/Center/Center_X", CenterXMsg);
  nh.getParam("/FIRA/Center/Center_Y", CenterYMsg);
  nh.getParam("/FIRA/Center/Inner", InnerMsg);
  nh.getParam("/FIRA/Center/Outer", OuterMsg);
  nh.getParam("/FIRA/Center/Front", FrontMsg);
  nh.getParam("/FIRA/Center/Camera_high", Camera_HighMsg);
/////////////////////////////////////BUTTONMSG////////////////////////////////////////
  nh.getParam("/FIRA/Parameterbutton", buttonmsg);

  //cout<<"read the YAML file"<<endl;
}
void InterfaceProc::Parameter_setting(const vision::parametercheck msg)
{
  paraMeterCheck = msg.checkpoint;
////////////////////////////////////如果有新的topic進來////////////////////////////
  if (paraMeterCheck != 0) {
    std::string temp = "rosparam dump " + param;
    const char *save = temp.c_str();
    system(save);
    paraMeterCheck = 0;
  }
  cout << "Parameter has change " << endl;
}
void InterfaceProc::SaveButton_setting(const vision::bin msg)
{

  SaveButton = msg.bin;
  Parameter_getting(1);
  //HSVmap();
}


InterfaceProc::InterfaceProc()
  : it_(nh)
{
  ros::NodeHandle n("~");
  //Parameter_getting(1);

  init_data();
  image_sub_ = it_.subscribe("/camera/image_raw", 1, &InterfaceProc::imageCb, this);
  //image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &InterfaceProc::imageCb, this);
  image_pub_threshold_ = it_.advertise("/camera/image", 1);//http://localhost:8080/stream?topic=/camera/image webfor /camera/image
  object_pub = nh.advertise<vision::Object>("/vision/object", 1);
  CenterDis_pub = nh.advertise<vision::dis>("/interface/CenterDis", 1);
  //white_pub  = nh.advertise<std_msgs::Int32MultiArray>("/vision/whiteRealDis",1);
  //black_pub  = nh.advertise<std_msgs::Int32MultiArray>("/vision/blackRealDis",1);
  //Two_point_pub = nh.advertise<vision::Two_point>("/interface/Two_point",1);
  s1 = nh.subscribe("interface/parameterbutton", 1000, &InterfaceProc::ParameterButtonCall, this);
  s2 = nh.subscribe("interface/color", 1000, &InterfaceProc::colorcall, this);
  s3 = nh.subscribe("interface/center", 1000, &InterfaceProc::centercall, this);
  s4 = nh.subscribe("interface/white", 1000, &InterfaceProc::whitecall, this);
  s5 = nh.subscribe("interface/camera", 1000, &InterfaceProc::cameracall, this);
  s6 = nh.subscribe("interface/black", 1000, &InterfaceProc::blackcall, this);
  s7 = nh.subscribe("interface/colorbutton", 1000, &InterfaceProc::colorbuttoncall, this);
  s8 = nh.subscribe("interface/scan", 1000, &InterfaceProc::scancall, this);
  s9 = nh.subscribe("interface/parametercheck", 1000, &InterfaceProc::Parameter_setting, this);
  s10 = nh.subscribe("interface/position", 1000, &InterfaceProc::positioncall, this);
  s11 = nh.subscribe("interface/bin_save", 1000, &InterfaceProc::SaveButton_setting, this);
  //cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
  //cv::Mat iframe;
  frame = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3 );
  frame_white = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3 );
  frame_black = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3 );
  CameraModels = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
  CenterModels = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
  ScanModels = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
  ColorModels = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
  WhiteModels = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
  BlackModels = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
  outputframe = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
  //imshow(OPENCV_WINDOW, outputframe);
}
InterfaceProc::~InterfaceProc()
{
  delete frame;
  delete frame_white;
  delete frame_black;
  delete CameraModels;
  delete CenterModels;
  delete ScanModels;
  delete ColorModels;
  delete BlackModels;
  delete WhiteModels;

  cv::destroyWindow(OPENCV_WINDOW);
}
/////////////////////////////////影像讀進來//////////////////////////////////////////
void InterfaceProc::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
//////////////////////Clone///////////////////////////////////
  cv::flip(cv_ptr->image, cv_ptr->image, 1);
  Main_frame = cv_ptr->image.clone();
  *frame = cv_ptr->image;
  *outputframe = *frame;
//////////////////////////////////////////////////////////////
  vision_path = ros::package::getPath("vision");
  //color_map = ColorFile();
  double ang_PI;
  for (int ang = 0 ; ang < 360; ang++) {
    ang_PI = ang * PI / 180;
    Angle_sin.push_back(sin(ang_PI));
    Angle_cos.push_back(cos(ang_PI));
  }


//////////////////////處理影像開始//////////////////////////////////////

  switch (buttonmsg) {
  case 1:
    *CameraModels = CameraModel(*frame);
    //cv::imshow(OPENCV_WINDOW, *CameraModels);
    outputframe = CameraModels;
    break;
  case 2:
    *CenterModels = CenterModel(*frame);
    //cv::imshow(OPENCV_WINDOW, *CenterModels);
    outputframe = CenterModels;
    break;
  case 3:
    *ScanModels = ScanModel(*frame);
    //cv::imshow(OPENCV_WINDOW, *ScanModels);
    outputframe = ScanModels;
    break;
  case 4:
    *ColorModels = ColorModel(*frame);
    //cv::imshow(OPENCV_WINDOW, *ColorModels);
    outputframe = ColorModels;
    break;
  case 5:
    *WhiteModels = White_Line(*frame);
    //cv::imshow(OPENCV_WINDOW, *WhiteModels);
    outputframe = WhiteModels;
    break;

  case 6:
    *BlackModels = Black_Line(*frame);
    //cv::imshow(OPENCV_WINDOW, *BlackModels);
    outputframe = BlackModels;
    break;
  case 7:

    break;
  }



  setMouseCallback(OPENCV_WINDOW, onMouse, NULL);

  if (onclick == 1) {
    vision::dis dis_msg;
    dis_msg.distance = Omni_distance(sqrt(pow(mousex - robotCenterX, 2) + pow(-1 * (mousey - robotCenterY), 2)));
    CenterDis_pub.publish(dis_msg);
    onclick = 0;
  }
  if (buttonmsg != 7) {
    sensor_msgs::ImagePtr thresholdMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *outputframe).toImageMsg();
    image_pub_threshold_.publish(thresholdMsg);
  }
  cv::waitKey(3);

}
///////////////////////////////////////////////////////////////////////
////////////////////////////////ColorModel/////////////////////////////
cv::Mat InterfaceProc::ColorModel(const cv::Mat iframe)
{
  static cv::Mat oframe(cv::Size(iframe.cols, iframe.rows), CV_8UC3);
  for (int i = 0; i < iframe.rows; i++) {
    for (int j = 0; j < iframe.cols; j++) {

      double B = iframe.data[(i * iframe.cols * 3) + (j * 3) + 0];
      double G = iframe.data[(i * iframe.cols * 3) + (j * 3) + 1];
      double R = iframe.data[(i * iframe.cols * 3) + (j * 3) + 2];
      double H, S, V;
      double Max = (max(R, G) > max(G, B)) ? max(R, G) : max(G, B); //max(R,G,B);
      double Min = (min(R, G) < min(G, B)) ? min(R, G) : min(G, B); //min(R,G,B);

      if (Max == Min)Max += 1;
      if (R == Max) {H = (G - B) * 60 / (Max - Min);}
      if (G == Max) {H = 120 + (B - R) * 60 / (Max - Min);}
      if (B == Max) {H = 240 + (R - G) * 60 / (Max - Min);}
      if (B == G && B == R) H = 0;
      if (H < 0) {H = H + 360;}
      S = (((Max - Min) * 100) / Max);
      if (Max == 0)S = 0;
      V = Max;
      //  usleep(300);
      switch (ColorModeMsg)
      {
      case 0:
        hmax = BallHSVBoxMsg[1];
        hmin = BallHSVBoxMsg[0];
        smax = BallHSVBoxMsg[3];
        smin = BallHSVBoxMsg[2];
        vmax = BallHSVBoxMsg[5];
        vmin = BallHSVBoxMsg[4];
        break;
      case 1:
        hmax = GreenHSVBoxMsg[1];
        hmin = GreenHSVBoxMsg[0];
        smax = GreenHSVBoxMsg[3];
        smin = GreenHSVBoxMsg[2];
        vmax = GreenHSVBoxMsg[5];
        vmin = GreenHSVBoxMsg[4];
        break;
      case 2:
        hmax = BlueHSVBoxMsg[1];
        hmin = BlueHSVBoxMsg[0];
        smax = BlueHSVBoxMsg[3];
        smin = BlueHSVBoxMsg[2];
        vmax = BlueHSVBoxMsg[5];
        vmin = BlueHSVBoxMsg[4];
        break;
      case 3:
        hmax = YellowHSVBoxMsg[1];
        hmin = YellowHSVBoxMsg[0];
        smax = YellowHSVBoxMsg[3];
        smin = YellowHSVBoxMsg[2];
        vmax = YellowHSVBoxMsg[5];
        vmin = YellowHSVBoxMsg[4];
        break;
      case 4:
        hmax = WhiteHSVBoxMsg[1];
        hmin = WhiteHSVBoxMsg[0];
        smax = WhiteHSVBoxMsg[3];
        smin = WhiteHSVBoxMsg[2];
        vmax = WhiteHSVBoxMsg[5];
        vmin = WhiteHSVBoxMsg[4];
        break;
      }

      vmin = vmin * 2.56;
      vmax = vmax * 2.56;

      if (ColorModeMsg == 0) {
        if (hmax > hmin) {
          if ((H <= hmax) && (H >= hmin) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin) ) {
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 197;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 149;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 0 ;
          } else {
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 0] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 0];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 1] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 1];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 2] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 2];
          }
        } else {
          if (((H <= hmax) || (H >= hmin)) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin) ) {
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 197;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 149;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 0 ;
          } else {
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 0] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 0];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 1] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 1];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 2] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 2];
          }
        }
      }
      else if (ColorModeMsg == 1) {
        if (hmax > hmin) {
          if ((H <= hmax) && (H >= hmin) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin) ) {
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 255;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 0;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 255 ;
          } else {
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 0] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 0];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 1] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 1];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 2] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 2];
          }
        } else {
          if (((H <= hmax) || (H >= hmin)) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin) ) {
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 255;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 0;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 255 ;
          } else {
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 0] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 0];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 1] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 1];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 2] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 2];
          }
        }
      }
      else if (ColorModeMsg == 2) {
        if (hmax > hmin) {
          if ((H <= hmax) && (H >= hmin) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin) ) {
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 127;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 183;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 224 ;
          } else {
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 0] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 0];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 1] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 1];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 2] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 2];
          }
        } else {
          if (((H <= hmax) || (H >= hmin)) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin) ) {
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 127;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 183;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 224 ;
          } else {
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 0] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 0];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 1] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 1];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 2] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 2];
          }
        }
      }
      else if (ColorModeMsg == 3) {
        if (hmax > hmin) {
          if ((H <= hmax) && (H >= hmin) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin) ) {
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 207;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 90;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 111 ;
          } else {
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 0] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 0];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 1] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 1];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 2] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 2];
          }
        } else {
          if (((H <= hmax) || (H >= hmin)) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin) ) {
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 207;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 90;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 111 ;
          } else {
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 0] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 0];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 1] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 1];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 2] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 2];
          }
        }
      }
      else if (ColorModeMsg == 4) {
        if (hmax > hmin) {
          if ((H <= hmax) && (H >= hmin) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin) ) {
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 100;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 0;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 255 ;
          } else {
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 0] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 0];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 1] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 1];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 2] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 2];
          }
        } else {
          if (((H <= hmax) || (H >= hmin)) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin) ) {
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 100;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 0 ;
            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 255 ;
          } else {
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 0] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 0];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 1] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 1];
            oframe.data[(i * iframe.cols * 3) + (j * 3) + 2] = iframe.data[(i * iframe.cols * 3) + (j * 3) + 2];
          }
        }
      }
    }
  }

  return oframe;
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////FPS///////////////////////////////////////////////////
cv::Mat InterfaceProc::CameraModel(const cv::Mat iframe)
{
  //if(0<fpsMsg<=100){}else{fpsMsg=25;}
  set_campara(fpsMsg);
  return iframe;
}
///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////CenterModel//////////////////////////////////
cv::Mat InterfaceProc::CenterModel(const cv::Mat iframe)
{
  int lengh = 30, x, y;
  static cv::Mat oframe(cv::Size(iframe.cols, iframe.rows), CV_8UC3);
  oframe = iframe;
  //cout<<CenterXMsg;
  if (0 < CenterXMsg < 600) {} else {CenterXMsg = 0; CenterYMsg = 0; InnerMsg = 0; OuterMsg = 0; FrontMsg = 0;} //avoid code dump

  robotCenterX = CenterXMsg; //iframe.cols*(CenterXMsg*1);
  robotCenterY = CenterYMsg; //iframe.rows*(CenterYMsg*1);

  circle(oframe, Point(robotCenterX, robotCenterY), 1, Scalar(0, 255, 0), 1);
  circle(oframe, Point(robotCenterX, robotCenterY), InnerMsg , Scalar(0, 0, 255), 1);
  circle(oframe, Point(robotCenterX, robotCenterY), OuterMsg , Scalar(0, 255, 0), 1);
  x = robotCenterX + lengh * cos(FrontMsg * PI / 180), y = robotCenterY - lengh * sin(FrontMsg * PI / 180);
  line(oframe, Point(robotCenterX, robotCenterY), Point(x, y), Scalar(255, 0, 255), 1);
  return oframe;
}
///////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////distance//////////////////////////////////////////
void onMouse(int Event, int x, int y, int flags, void* param)
{
  if (Event == CV_EVENT_LBUTTONDOWN) {
    mousex = x;
    mousey = y;
    onclick = 1;
  }
}
double InterfaceProc::camera_f(double Omni_pixel)
{
  double m = (Omni_pixel * 0.0099) / 60;    // m = H1/H0 = D1/D0    D0 + D1 = 180
  double D0 = 180 / (1 + m);                // D1 = m   *D0
  double D1 = 180 / (1 + (1 / m));          // D0 = 1/m *D1
  double f = 1 / (1 / D0 + 1 / D1);
  //ROS_INFO("m = %f D0 = %f D1 = %f F = %f",m,D0,D1,f);
  return D1;
}
double InterfaceProc::Omni_distance(double pixel_dis)
{
  double Z = -1 * Camera_HighMsg; //Camera_HighMsg=65;
  //double c  =  D0/2;
  double c = 83.125;
  double b = c * 0.8722;
  double f = camera_f(OuterMsg * 2 * 0.9784);
  double r = atan2(f, pixel_dis * 0.0099);
  double dis = Z * (pow(b, 2) - pow(c, 2)) * cos(r) / ((pow(b, 2) + pow(c, 2)) * sin(r) - 2 * b * c) * 0.1;
  if (dis < 0 || dis > 999) {dis = 999;}
  //ROS_INFO("%f %f %f %f",Z,c,r,dis);
  return dis;
}
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////掃描參數(Scan)//////////////////////////////////
//掃描點座標調整
//修正超出圖片的點座標
int Frame_Area(int coordinate, int range)
{
  if (coordinate < 0) coordinate = 0;
  else if (coordinate >= range) coordinate = range - 1;
  return coordinate;
}
//角度調整
//修正大於或小於360的角度
int Angle_Adjustment(int angle)
{
  if (angle < 0) return angle + 360;
  else if (angle >= 360) return angle - 360;
  else return angle;
}
//角度間隔
//middle start 到 far start 之間　Angle near gap的值為1/2
//far start 之外 Angle near gap的值為1/4
int InterfaceProc::Angle_Interval(int radius)
{
  if (radius <= Magn_Middle_StartMsg) return Angle_Near_GapMsg;
  else if (radius > Magn_Middle_StartMsg && radius <= Magn_Far_StartMsg) return Angle_Near_GapMsg / 2;
  else return Angle_Near_GapMsg / 4;
}
cv::Mat InterfaceProc::ScanModel(const cv::Mat iframe)
{
  static cv::Mat oframe(cv::Size(iframe.cols, iframe.rows), CV_8UC3);
  oframe = iframe;
  int Unscaned_Area[6] = {0};
  int x, y;

  Unscaned_Area[0] = Angle_Adjustment(Dont_Search_Angle_1Msg - Angle_range_1Msg);
  Unscaned_Area[1] = Angle_Adjustment(Dont_Search_Angle_1Msg + Angle_range_1Msg);
  Unscaned_Area[2] = Angle_Adjustment(Dont_Search_Angle_2Msg - Angle_range_2_3Msg);
  Unscaned_Area[3] = Angle_Adjustment(Dont_Search_Angle_2Msg + Angle_range_2_3Msg);
  Unscaned_Area[4] = Angle_Adjustment(Dont_Search_Angle_3Msg - Angle_range_2_3Msg);
  Unscaned_Area[5] = Angle_Adjustment(Dont_Search_Angle_3Msg + Angle_range_2_3Msg);

  for (int radius = Magn_Near_StartMsg ; radius <= Magn_Far_EndMsg ; radius += Magn_Near_GapMsg) {
    for (int angle = 0 ; angle < 360 ;) {
      //略過柱子
      if (angle >= Unscaned_Area[0] && angle <= Unscaned_Area[1] ||
          angle >= Unscaned_Area[2] && angle <= Unscaned_Area[3] ||
          angle >= Unscaned_Area[4] && angle <= Unscaned_Area[5]) {
        angle += Angle_Interval(radius);
        continue;
      }
      //掃描點的座標值
      x = Frame_Area(robotCenterX + radius * cos(angle * PI / 180), oframe.cols);
      y = Frame_Area(robotCenterY - radius * sin(angle * PI / 180), oframe.rows);
      //畫掃描點
      oframe.data[(y * oframe.cols + x) * 3 + 0] = 0;
      oframe.data[(y * oframe.cols + x) * 3 + 1] = 255;
      oframe.data[(y * oframe.cols + x) * 3 + 2] = 0;
      angle += Angle_Interval(radius);
    }
  }
  return oframe;
}
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////White_Line///////////////////////////////
cv::Mat InterfaceProc::White_Line(const cv::Mat iframe)
{
  static cv::Mat oframe(cv::Size(iframe.cols, iframe.rows), CV_8UC3);
  oframe = iframe;

  for (int i = 0; i < oframe.rows; i++) {
    for (int j = 0; j < oframe.cols; j++) {
      unsigned char gray = ( oframe.data[(i * oframe.cols * 3) + (j * 3) + 0]
                             + oframe.data[(i * oframe.cols * 3) + (j * 3) + 1]
                             + oframe.data[(i * oframe.cols * 3) + (j * 3) + 2]) / 3;
      if (gray < WhiteGrayMsg) {
        oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 0;
        oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 0;
        oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 0;
      } else {
        oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 255;
        oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 255;
        oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 255;
      }
    }
  }
  for (double angle = FrontMsg; angle < 360+FrontMsg; angle = angle + WhiteAngleMsg) {
    double angle_be = angle;

    if (angle_be > 360)angle_be -= 360;
    double x_ = cos((angle_be * PI) / 180); //Angle_cos[angle_be];
    double y_ = sin((angle_be * PI) / 180); //Angle_sin[angle_be];

    for (int r = InnerMsg; r <= OuterMsg; r++) {

      int x=r*x_ , y = r*y_;
      if ( oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x) * 3 + 0] == 255
           && oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x) * 3 + 1] == 255
           && oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x) * 3 + 2] == 255) {
        break;
      } else {
        oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x) * 3 + 0] = 0;
        oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x) * 3 + 1] = 0;
        oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x) * 3 + 2] = 255;
      }
    }
  }


  line(oframe, Point(CenterXMsg, CenterYMsg - InnerMsg), Point(CenterXMsg, CenterYMsg + InnerMsg), Scalar(0, 255, 0), 1);
  line(oframe, Point(CenterXMsg - InnerMsg, CenterYMsg), Point(CenterXMsg + InnerMsg, CenterYMsg), Scalar(0, 255, 0), 1);
  circle(oframe, Point(CenterXMsg, CenterYMsg), InnerMsg, Scalar(0, 255, 0), 0);
  circle(oframe, Point(CenterXMsg, CenterYMsg), OuterMsg, Scalar(0, 255, 0), 0);

  return oframe;
}

////////////////////////////////////////////////////////////////////////
///////////////////////////////BlackItem////////////////////////////////
cv::Mat InterfaceProc::Black_Line(const cv::Mat iframe)
{
  static cv::Mat oframe(cv::Size(iframe.cols, iframe.rows), CV_8UC3);
  oframe = iframe;

  for (int i = 0; i < oframe.rows; i++) {
    for (int j = 0; j < oframe.cols; j++) {
      unsigned char gray = ( oframe.data[(i * oframe.cols * 3) + (j * 3) + 0]
                             + oframe.data[(i * oframe.cols * 3) + (j * 3) + 1]
                             + oframe.data[(i * oframe.cols * 3) + (j * 3) + 2]) / 3;
      if (gray < BlackGrayMsg) {
        oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 0;
        oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 0;
        oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 0;
      } else {
        oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 255;
        oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 255;
        oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 255;
      }
    }
  }
  for (double angle = FrontMsg; angle < FrontMsg+360; angle = angle + BlackAngleMsg) {
    double angle_be = angle;
    if (angle_be > 360)angle_be -= 360;

double x_ = cos((angle_be * PI) / 180); //Angle_cos[angle_be];
    double y_ = sin((angle_be * PI) / 180); //Angle_sin[angle_be];
    for (int r = InnerMsg; r <= OuterMsg; r++) {
      int x=r*x_ , y = r*y_;
      if ( oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x) * 3 + 0] == 0
           && oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x) * 3 + 1] == 0
           && oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x) * 3 + 2] == 0) {
        break;
      } else {
        oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x) * 3 + 0] = 0;
        oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x) * 3 + 1] = 0;
        oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x) * 3 + 2] = 255;
      }
    }
  }
  line(oframe, Point(CenterXMsg, CenterYMsg - InnerMsg), Point(CenterXMsg, CenterYMsg + InnerMsg), Scalar(0, 255, 0), 1);
  line(oframe, Point(CenterXMsg - InnerMsg, CenterYMsg), Point(CenterXMsg + InnerMsg, CenterYMsg), Scalar(0, 255, 0), 1);
  circle(oframe, Point(CenterXMsg, CenterYMsg), InnerMsg, Scalar(0, 255, 0), 0);
  circle(oframe, Point(CenterXMsg, CenterYMsg), OuterMsg, Scalar(0, 255, 0), 0);

  return oframe;

}
/*
//////////////////////////////monitor///////////////////////////////////////////////////
void InterfaceProc::object_Item_reset(object_Item &obj_){
  obj_.dis_max = 0;
  obj_.dis_min = 0;
  obj_.ang_max = 0;
  obj_.ang_min = 0;
  obj_.x = 0;
  obj_.y = 0;
  obj_.angle = 0;
  obj_.distance = 0;
  obj_.size = 0;
  obj_.LR = "null";

}
void InterfaceProc::objectdet_change(Mat &frame_, int color, object_Item &obj_item){
  int x,y;
  int x_,y_;
  int object_size;
  int dis,ang;

  frame_ = Mat(Size(Main_frame.cols,Main_frame.rows),CV_8UC3,Scalar(0,0,0));

  find_point.clear();
  object_Item_reset(FIND_Item);

  for(int distance = search_start ; distance <= search_end ; distance += search_distance){
    for(int angle = 0;angle < 360;){
      if(angle >= dont_angle[0] && angle <= dont_angle[1] ||
         angle >= dont_angle[2] && angle <= dont_angle[3] ||
         angle >= dont_angle[4] && angle <= dont_angle[5]) {
        angle += Angle_Interval(distance);
        continue;
      }
      object_size = 0;
      FIND_Item.size = 0;

      x_= distance*Angle_cos[angle];
      y_= distance*Angle_sin[angle];

      x = Frame_Area(center_x+x_,frame_.cols);
      y = Frame_Area(center_y-y_,frame_.rows);

      unsigned char B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
      unsigned char G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
      unsigned char R = Main_frame.data[(y*Main_frame.cols+x)*3+2];
      if(color_map[R+(G<<8)+(B<<16)] & color && frame_.data[(y*frame_.cols+x)*3+0] == 0){
        Mark_point(frame_, distance, angle ,x , y, object_size, color);
        FIND_Item.dis_max = distance;
        FIND_Item.dis_min = distance;
        FIND_Item.ang_max = angle;
        FIND_Item.ang_min = angle;
        while(!find_point.empty()){
          dis = find_point.front();
          find_point.pop_front();

          ang = find_point.front();
          find_point.pop_front();

          object_compare(dis, ang);
          find_around(frame_, dis, ang, object_size, color);
        }
        FIND_Item.size = object_size;
      }

      find_point.clear();

      if(FIND_Item.size > obj_item.size){
        obj_item = FIND_Item;
      }

      angle += Angle_Interval(distance);
    }
  }

  find_object_point(obj_item,color);

//  if(color == REDITEM){
//    draw_ellipse(frame_,Red_Item);
//    cv::imshow("R", frame_);
//    cv::waitKey(10);
//  }
//  if(color == YELLOWITEM){
//    draw_ellipse(frame_,Yellow_Item);
//    cv::imshow("Y", frame_);
//    cv::waitKey(1);
//  }
//  if(color == BLUEITEM){
//    draw_ellipse(frame_,Blue_Item);
//    cv::imshow("B", frame_);
//    cv::waitKey(10);
//  }

  draw_ellipse(Main_frame,Red_Item,REDITEM);
  draw_ellipse(Main_frame,Yellow_Item,YELLOWITEM);
  draw_ellipse(Main_frame,Blue_Item,BLUEITEM);

  if(Red_Item.x!=0){Draw_cross(Main_frame,'R');}
  if(Blue_Item.x!=0){Draw_cross(Main_frame,'B');}
  if(Yellow_Item.x!=0){Draw_cross(Main_frame,'Y');}
}
//////////////////////////////////////////////////////////
void InterfaceProc::creat_Obstclemap(Mat &frame_, int color)
{
  int x,y;
  int x_,y_;
  int object_size;
  int dis,ang;

  frame_ = Mat(Size(Main_frame.cols,Main_frame.rows),CV_8UC3,Scalar(0,0,0));

  for(int distance = search_start ; distance <= search_end ; distance++){
    for(int angle = 0; angle < 360; angle++){

      if(angle >= dont_angle[0] && angle <= dont_angle[1] ||
         angle >= dont_angle[2] && angle <= dont_angle[3] ||
         angle >= dont_angle[4] && angle <= dont_angle[5]) {
        angle += Angle_Interval(distance);
        continue;
      }

      x_= distance*Angle_cos[angle];
      y_= distance*Angle_sin[angle];

      x = Frame_Area(center_x+x_,frame_.cols);
      y = Frame_Area(center_y-y_,frame_.rows);

      unsigned char B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
      unsigned char G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
      unsigned char R = Main_frame.data[(y*Main_frame.cols+x)*3+2];

      if(!(color_map[R+(G<<8)+(B<<16)] ^ color)){
        Mark_point(frame_, distance, angle ,x , y, object_size, color);
      }
      angle++;
    }
  }

  cv::dilate(frame_, frame_, Mat(), Point(), 2);
  cv::erode(frame_, frame_, Mat(), Point(), 3);
  cv::dilate(frame_, frame_, Mat(), Point(), 1);
}
void InterfaceProc::creat_FIRA_map(Mat &frame_input , Mat &frame_output)
{
  frame_output = Mat(Size(600,600),CV_8UC1,Scalar(0,0,0));

  int frame_center_x = frame_output.cols / 2;
  int frame_center_y = frame_output.rows / 2;

  int x,y;
  int x_,y_;

  double dis;
  int angle_;

  Point points[360];

  int distance_get = 0;

  for(int angle = 0; angle < 360; angle++){
    for(int distance = search_start ; distance <= search_middle ; distance += search_distance){
      x_= distance*Angle_cos[angle];
      y_= distance*Angle_sin[angle];

      x = Frame_Area(center_x+x_,frame_input.cols);
      y = Frame_Area(center_y-y_,frame_input.rows);

      if(frame_input.data[(y*frame_input.cols+x)*3+0]==255){
        distance_get = 1;

        dis = Omni_distance(distance);

        angle_ = Angle_Adjustment(angle-center_front+90);

        x_= dis*Angle_cos[angle_];
        y_= dis*Angle_sin[angle_];

        x = Frame_Area(frame_center_x+x_,frame_output.cols);
        y = Frame_Area(frame_center_y-y_,frame_output.rows);

        points[angle] = Point(x, y);
        break;
      }
    }
    if(distance_get == 0){
      dis = Omni_distance(search_end);

      angle_ = Angle_Adjustment(angle-center_front+90);

      x_= dis*Angle_cos[angle_];
      y_= dis*Angle_sin[angle_];

      x = Frame_Area(frame_center_x+x_,frame_output.cols);
      y = Frame_Area(frame_center_y-y_,frame_output.rows);

      points[angle] = Point(x, y);
    }
    distance_get = 0;
  }

  for(int i=0 ;i<360;i++){
    line(frame_output, points[i], points[Angle_Adjustment(i+1)], 255, 1);
  }
}

void InterfaceProc::objectdet_Obstacle(Mat &frame_, int color, object_Item *obj_item){
  int x,y;
  int x_,y_;
  int object_size;
  int dis,ang;

  frame_ = Mat(Size(Main_frame.cols,Main_frame.rows),CV_8UC3,Scalar(0,0,0));

  find_point.clear();

  object_Item_reset(FIND_Item);

  for(int distance = search_start ; distance <= search_middle ; distance += search_distance){
    for(int angle = 0; angle < 360;){
      object_size = 0;
      FIND_Item.size = 0;

      x_= distance*Angle_cos[angle];
      y_= distance*Angle_sin[angle];

      x = Frame_Area(center_x+x_,frame_.cols);
      y = Frame_Area(center_y-y_,frame_.rows);

      unsigned char B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
      unsigned char G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
      unsigned char R = Main_frame.data[(y*Main_frame.cols+x)*3+2];

      if(Obstaclemap.data[(y*frame_.cols+x)*3+0]==255 && frame_.data[(y*frame_.cols+x)*3+0] == 0){
        Mark_point(frame_, distance, angle ,x , y, object_size, color);
        FIND_Item.dis_max = distance;
        FIND_Item.dis_min = distance;
        FIND_Item.ang_max = angle;
        FIND_Item.ang_min = angle;
        while(!find_point.empty()){
          dis = find_point.front();
          find_point.pop_front();

          ang = find_point.front();
          find_point.pop_front();

          object_compare(dis, ang);
          find_around(frame_, dis, ang, object_size, color);
        }

        FIND_Item.size = object_size;
      }

      find_point.clear();

      if(FIND_Item.size > (obj_item+0)->size && FIND_Item.size > 10){
        *(obj_item+0) = FIND_Item;
        if(FIND_Item.size > (obj_item+1)->size){
          *(obj_item+0) = *(obj_item+1);
          *(obj_item+1) = FIND_Item;
          if(FIND_Item.size > (obj_item+2)->size){
            *(obj_item+1) = *(obj_item+2);
            *(obj_item+2) = FIND_Item;
            if(FIND_Item.size > (obj_item+3)->size){
              *(obj_item+2) = *(obj_item+3);
              *(obj_item+3) = FIND_Item;
              if(FIND_Item.size > (obj_item+4)->size){
                *(obj_item+3) = *(obj_item+4);
                *(obj_item+4) = FIND_Item;
              }
            }
          }
        }
      }
      angle += Angle_Interval(distance);
    }
  }
  find_object_point(*(obj_item+0),color);
  find_object_point(*(obj_item+1),color);
  find_object_point(*(obj_item+2),color);
  find_object_point(*(obj_item+3),color);
  find_object_point(*(obj_item+4),color);
}

void InterfaceProc::Mark_point(Mat &frame_, int distance, int angle, int x, int y, int &size, int color){
  frame_.data[(y*frame_.cols+x)*3+0] = 255;
  frame_.data[(y*frame_.cols+x)*3+1] = 255;
  frame_.data[(y*frame_.cols+x)*3+2] = 255;
  find_point.push_back(distance);
  find_point.push_back(angle);
  size += 1;
}

void InterfaceProc::find_around(Mat &frame_, int distance ,int angle, int &size, int color){
  int x,y;
  int x_,y_;
  int dis_f,ang_f;
  double angle_f;

  for(int i=-1 ; i<2 ; i++){
    for(int j=-1 ; j<2 ; j++){
      dis_f = distance + i*search_distance;

      if(dis_f < search_start) dis_f = search_start;

      if(color == REDITEM || color == BLUEITEM || color == YELLOWITEM){
        dis_f = Frame_Area(dis_f,search_end);
      }else if(color == OBSTACLEITEM){
        dis_f = Frame_Area(dis_f,search_middle);
      }

      ang_f = angle + j*Angle_Interval(dis_f);

      while(Angle_Adjustment(ang_f) > dont_angle[0] && Angle_Adjustment(ang_f) < dont_angle[1] ||
         Angle_Adjustment(ang_f) > dont_angle[2] && Angle_Adjustment(ang_f) < dont_angle[3] ||
         Angle_Adjustment(ang_f) > dont_angle[4] && Angle_Adjustment(ang_f) < dont_angle[5]) {
        ang_f += j*Angle_Interval(dis_f);
      }

      angle_f = Angle_Adjustment(ang_f);

      x_= dis_f*Angle_cos[angle_f];
      y_= dis_f*Angle_sin[angle_f];

      x = Frame_Area(center_x+x_,frame_.cols);
      y = Frame_Area(center_y-y_,frame_.rows);

      unsigned char B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
      unsigned char G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
      unsigned char R = Main_frame.data[(y*Main_frame.cols+x)*3+2];

      if(color == REDITEM || color == BLUEITEM || color == YELLOWITEM){
        if(color_map[R+(G<<8)+(B<<16)] & color && frame_.data[(y*frame_.cols+x)*3+0] == 0){
          Mark_point(frame_, dis_f, ang_f ,x , y, size, color);
        }
      }else if(color == OBSTACLEITEM){
        if(Obstaclemap.data[(y*frame_.cols+x)*3+0]==255 && frame_.data[(y*frame_.cols+x)*3+0] == 0){
          Mark_point(frame_, dis_f, ang_f ,x , y, size, color);
        }
      }
    }
  }
}
void InterfaceProc::find_object_point(object_Item &obj_, int color){
  int x,y;
  int x_,y_;

  int angle_,distance_;
  int angle_range;
  int find_angle;

  unsigned char B,G,R;
/*
  if(color == REDITEM){
    angle_= Angle_Adjustment((obj_.ang_max + obj_.ang_min)/2);
    distance_ = (obj_.dis_max + obj_.dis_min) / 2;
    find_angle = Angle_Adjustment(angle_);
    for(int distance = obj_.dis_min ; distance <= obj_.dis_max ; distance++){
      x_= distance_*Angle_cos[find_angle];
      y_= distance_*Angle_sin[find_angle];

      x = Frame_Area(center_x+x_,Main_frame.cols);
      y = Frame_Area(center_y-y_,Main_frame.rows);

      B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
      G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
      R = Main_frame.data[(y*Main_frame.cols+x)*3+2];

      if(color_map[R+(G<<8)+(B<<16)] & color){
        obj_.x = x;
        obj_.y = y;
        obj_.distance = sqrt(pow(x_,2)+pow(y_,2));
        angle_ = find_angle;
        break;
      }
    }
  }else

  if(color == REDITEM || color == BLUEITEM || color == YELLOWITEM){
    angle_= Angle_Adjustment((obj_.ang_max + obj_.ang_min)/2);
    angle_range = 0.7*Angle_Adjustment((obj_.ang_max - obj_.ang_min)/2);

    for(int distance = obj_.dis_min ; distance <= obj_.dis_max ; distance++){
      for(int angle = 0 ; angle <= angle_range ; angle++){
        find_angle = Angle_Adjustment(angle_ + angle);

        x_= distance*Angle_cos[find_angle];
        y_= distance*Angle_sin[find_angle];

        x = Frame_Area(center_x+x_,Main_frame.cols);
        y = Frame_Area(center_y-y_,Main_frame.rows);

        B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
        G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
        R = Main_frame.data[(y*Main_frame.cols+x)*3+2];

        if(color_map[R+(G<<8)+(B<<16)] & color){
          obj_.x = x;
          obj_.y = y;
          obj_.distance = sqrt(pow(x_,2)+pow(y_,2));
          angle_ = find_angle;
          break;
        }

        find_angle = Angle_Adjustment(angle_ - angle);

        x_= distance*Angle_cos[find_angle];
        y_= distance*Angle_sin[find_angle];

        x = Frame_Area(center_x+x_,Main_frame.cols);
        y = Frame_Area(center_y-y_,Main_frame.rows);

        B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
        G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
        R = Main_frame.data[(y*Main_frame.cols+x)*3+2];

        if(color_map[R+(G<<8)+(B<<16)] & color){
          obj_.x = x;
          obj_.y = y;
          obj_.distance = sqrt(pow(x_,2)+pow(y_,2));
          angle_ = find_angle;
          break;
        }
      }
      if(obj_.distance != 0){
          break;
      }
    }
  }else if(color == OBSTACLEITEM){
    angle_= Angle_Adjustment((obj_.ang_max + obj_.ang_min)/2);
    angle_range = Angle_Adjustment((obj_.ang_max - obj_.ang_min)/2);

    for(int distance = obj_.dis_min ; distance <= obj_.dis_max ; distance++){
      for(int angle = 0 ; angle <= angle_range ; angle++){
        find_angle = Angle_Adjustment(angle_ + angle);

        x_= distance*Angle_cos[find_angle];
        y_= distance*Angle_sin[find_angle];

        x = Frame_Area(center_x+x_,Main_frame.cols);
        y = Frame_Area(center_y-y_,Main_frame.rows);

        B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
        G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
        R = Main_frame.data[(y*Main_frame.cols+x)*3+2];

        if(!(color_map[R+(G<<8)+(B<<16)] ^ color)){
          obj_.x = x;
          obj_.y = y;
          obj_.distance = sqrt(pow(x_,2)+pow(y_,2));
          angle_ = find_angle;
          break;
        }

        find_angle = Angle_Adjustment(angle_ - angle);

        x_= distance*Angle_cos[find_angle];
        y_= distance*Angle_sin[find_angle];

        x = Frame_Area(center_x+x_,Main_frame.cols);
        y = Frame_Area(center_y-y_,Main_frame.rows);

        B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
        G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
        R = Main_frame.data[(y*Main_frame.cols+x)*3+2];

        if(!(color_map[R+(G<<8)+(B<<16)] ^ color)){
          obj_.x = x;
          obj_.y = y;
          obj_.distance = sqrt(pow(x_,2)+pow(y_,2));
          angle_ = find_angle;
          break;
        }
      }
      if(obj_.distance != 0){
          break;
      }
    }
  }

  if(Angle_Adjustment(angle_-center_front) < 180){
    obj_.LR = "Left";
    obj_.angle = Angle_Adjustment(angle_-center_front);
  }else{
    obj_.LR = "Right";
    obj_.angle = Angle_Adjustment(angle_-center_front)-360;
  }
}
void InterfaceProc::draw_ellipse(Mat &frame_, object_Item &obj_,int color){
  ellipse(frame_, Point(center_x,center_y), Size(obj_.dis_min,obj_.dis_min), 0, 360-obj_.ang_max, 360-obj_.ang_min, Scalar(255,255,0), 1);
  ellipse(frame_, Point(center_x,center_y), Size(obj_.dis_max,obj_.dis_max), 0, 360-obj_.ang_max, 360-obj_.ang_min, Scalar(255,255,0), 1);
  draw_Line(frame_, obj_.dis_max, obj_.dis_min, obj_.ang_max);
  draw_Line(frame_, obj_.dis_max, obj_.dis_min, obj_.ang_min);
  circle(frame_, Point(obj_.x,obj_.y), 2, Scalar(0,0,255), -1);

  vision::Two_point Two_point_msg;

  int x_1,y_1,x_2,y_2,x_3,y_3,x_4,y_4;
  double blue_angle_max;
  double blue_angle_min;
  double yellow_angle_max;
  double yellow_angle_min;
  int x[4],y[4];

  if(color = BLUEITEM){
     blue_angle_max = Angle_Adjustment(Blue_Item.ang_max);
     blue_angle_min = Angle_Adjustment(Blue_Item.ang_min);

      /*x_1= Blue_Item.dis_min*Angle_cos[blue_angle_max];
      y_1= Blue_Item.dis_min*Angle_sin[blue_angle_max];

      x_2= Blue_Item.dis_min*Angle_cos[blue_angle_min];
      y_2= Blue_Item.dis_min*Angle_sin[blue_angle_min];

      x[0] = Frame_Area(center_x+x_1,frame_.cols);
      y[0] = Frame_Area(center_y-y_1,frame_.rows);

      x[1] = Frame_Area(center_x+x_2,frame_.cols);
      y[1] = Frame_Area(center_y-y_2,frame_.rows);
      //line(frame_, Point(x[0],y[0]), Point(x[0],y[0]), Scalar(0,0,0), 10);
      //line(frame_, Point(x[1],y[1]), Point(x[1],y[1]), Scalar(0,0,0), 10);

      Two_point_msg.blue_x_1 = x[0];
      Two_point_msg.blue_x_1 = y[0];
      Two_point_msg.blue_x_2 = x[1];
      Two_point_msg.blue_y_2 = y[1];
      Two_point_msg.blue_dis = Blue_Item.dis_min;
      Two_point_msg.blue_ang1 = blue_angle_max;
      Two_point_msg.blue_ang2 = blue_angle_min;}

  if(color = YELLOWITEM){
      yellow_angle_max = Angle_Adjustment(Yellow_Item.ang_max);
      yellow_angle_min = Angle_Adjustment(Yellow_Item.ang_min);

      /*x_3= Yellow_Item.dis_min*Angle_cos[yellow_angle_max];
      y_3= Yellow_Item.dis_min*Angle_sin[yellow_angle_max];

      x_4= Yellow_Item.dis_min*Angle_cos[yellow_angle_min];
      y_4= Yellow_Item.dis_min*Angle_sin[yellow_angle_min];

      x[2] = Frame_Area(center_x+x_3,frame_.cols);
      y[2] = Frame_Area(center_y-y_3,frame_.rows);

      x[3] = Frame_Area(center_x+x_4,frame_.cols);
      y[3] = Frame_Area(center_y-y_4,frame_.rows);

      //line(frame_, Point(x[2],y[2]), Point(x[2],y[2]), Scalar(0,0,0), 10);
      //line(frame_, Point(x[3],y[3]), Point(x[3],y[3]), Scalar(0,0,0), 10);

      Two_point_msg.yellow_x_1 = x[2];
      Two_point_msg.yellow_x_1 = y[2];
      Two_point_msg.yellow_x_2 = x[3];
      Two_point_msg.yellow_y_2 = y[3];
      Two_point_msg.yellow_dis = Yellow_Item.dis_min;
      Two_point_msg.yellow_ang1 = yellow_angle_max;
      Two_point_msg.yellow_ang2 = yellow_angle_min;}

  Two_point_pub.publish(Two_point_msg);


  }


void InterfaceProc::draw_Line(Mat &frame_, int obj_distance_max, int obj_distance_min, int obj_angle){
  int x_,y_;
  double angle_f;
  int x[2],y[2];

  angle_f = Angle_Adjustment(obj_angle);

  x_= obj_distance_min*Angle_cos[angle_f];
  y_= obj_distance_min*Angle_sin[angle_f];

  x[0] = Frame_Area(center_x+x_,frame_.cols);
  y[0] = Frame_Area(center_y-y_,frame_.rows);

  x_= obj_distance_max*Angle_cos[angle_f];
  y_= obj_distance_max*Angle_sin[angle_f];

  x[1] = Frame_Area(center_x+x_,frame_.cols);
  y[1] = Frame_Area(center_y-y_,frame_.rows);

  line(frame_, Point(x[0],y[0]), Point(x[1],y[1]), Scalar(255,255,0), 1);

}
void InterfaceProc::Draw_cross(cv::Mat &frame_,char color){
  std::string R_X;
  std::string R_Y;
  std::stringstream R_X_out;
  std::stringstream R_Y_out;

  std::string B_X;
  std::string B_Y;
  std::stringstream B_X_out;
  std::stringstream B_Y_out;

  std::string Y_X;
  std::string Y_Y;
  std::stringstream Y_X_out;
  std::stringstream Y_Y_out;

  switch(color){
    case 'R':
      for(int i=-2;i<=2;i++){
        frame_.data[((Red_Item.y+i)*frame_.cols*3)+((Red_Item.x+0)*3)+0] = 0;
        frame_.data[((Red_Item.y+i)*frame_.cols*3)+((Red_Item.x+0)*3)+1] = 255;
        frame_.data[((Red_Item.y+i)*frame_.cols*3)+((Red_Item.x+0)*3)+2] = 0;
      }
      for(int j=-2;j<=2;j++){
        frame_.data[((Red_Item.y+0)*frame_.cols*3)+((Red_Item.x+j)*3)+0] = 0;
        frame_.data[((Red_Item.y+0)*frame_.cols*3)+((Red_Item.x+j)*3)+1] = 255;
        frame_.data[((Red_Item.y+0)*frame_.cols*3)+((Red_Item.x+j)*3)+2] = 0;
      }
      R_X_out << Red_Item.x;
      R_Y_out << Red_Item.y;
      R_X = R_X_out.str();
      R_Y = R_Y_out.str();
      cv::putText(frame_, "R("+R_X+","+R_Y+")", Point(Red_Item.x,Red_Item.y), 0, 0.5, Scalar(0,0,255),1);
      break;
    case 'B':
      for(int i=-2;i<=2;i++){
        frame_.data[((Blue_Item.y+i)*frame_.cols*3)+((Blue_Item.x+0)*3)+0] = 0;
        frame_.data[((Blue_Item.y+i)*frame_.cols*3)+((Blue_Item.x+0)*3)+1] = 0;
        frame_.data[((Blue_Item.y+i)*frame_.cols*3)+((Blue_Item.x+0)*3)+2] = 255;
      }
      for(int j=-2;j<=2;j++){
        frame_.data[((Blue_Item.y+0)*frame_.cols*3)+((Blue_Item.x+j)*3)+0] = 0;
        frame_.data[((Blue_Item.y+0)*frame_.cols*3)+((Blue_Item.x+j)*3)+1] = 0;
        frame_.data[((Blue_Item.y+0)*frame_.cols*3)+((Blue_Item.x+j)*3)+2] = 255;
      }
      B_X_out << Blue_Item.x;
      B_Y_out << Blue_Item.y;
      B_X = B_X_out.str();
      B_Y = B_Y_out.str();
      cv::putText(frame_, "B("+B_X+","+B_Y+")", Point(Blue_Item.x,Blue_Item.y), 0, 0.5, Scalar(255,0,0),1);
      break;
    case 'Y':
      for(int i=-2;i<=2;i++){
        frame_.data[((Yellow_Item.y+i)*frame_.cols*3)+((Yellow_Item.x+0)*3)+0] = 255;
        frame_.data[((Yellow_Item.y+i)*frame_.cols*3)+((Yellow_Item.x+0)*3)+1] = 0;
        frame_.data[((Yellow_Item.y+i)*frame_.cols*3)+((Yellow_Item.x+0)*3)+2] = 0;
      }
      for(int j=-2;j<=2;j++){
        frame_.data[((Yellow_Item.y+0)*frame_.cols*3)+((Yellow_Item.x+j)*3)+0] = 255;
        frame_.data[((Yellow_Item.y+0)*frame_.cols*3)+((Yellow_Item.x+j)*3)+1] = 0;
        frame_.data[((Yellow_Item.y+0)*frame_.cols*3)+((Yellow_Item.x+j)*3)+2] = 0;
      }
      Y_X_out << Yellow_Item.x;
      Y_Y_out << Yellow_Item.y;
      Y_X = Y_X_out.str();
      Y_Y = Y_Y_out.str();
      cv::putText(frame_, "Y("+Y_X+"," ")", Point(Yellow_Item.x,Yellow_Item.y), 0, 0.5, Scalar(0,255,255),1);
      break;
  }
}
void InterfaceProc::RGBtoHSV_maxmin(double &Rnew, double &Gnew, double &Bnew, double &HSVmax, double &HSVmin)
{
  HSVmax = max(max(Rnew,Gnew),Bnew);
  HSVmin = min(min(Rnew,Gnew),Bnew);
}
double InterfaceProc::RGBtoHSV_H(double Rnew, double Gnew, double Bnew, double HSVmax, double HSVmin)
{
    double range = HSVmax-HSVmin;
    double H_;
    if(range == 0){
        return 0;
    }else if(HSVmax == Rnew){
        H_ = 60.0*((Gnew-Bnew)/range);
    }else if(HSVmax == Gnew){
        H_ = 60.0*((Bnew-Rnew)/range + 2);
    }else if(HSVmax == Bnew){
        H_ = 60.0*((Rnew-Gnew)/range + 4);
    }
    if(H_ < 0) H_ += 360;
    return H_;
}
double InterfaceProc::RGBtoHSV_S(double HSVmax, double HSVmin)
{
    double range = HSVmax-HSVmin;
    if(range == 0){
        return 0;
    }else{
        return range/HSVmax*255;
    }
}
////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////HSVmap////////////////////////////////
void InterfaceProc::HSVmap()
{
     unsigned char *HSVmap = new unsigned char[256*256*256];
    for(int b=0;b<256;b++){
        for(int g=0;g<256;g++){
            for(int r=0;r<256;r++){
                double R,G,B,H_sum,S_sum,V_sum;
                B = b/255.0;
                G = g/255.0;
                R = r/255.0;
                double Max = (max(R,G)>max(G,B))?max(R,G):max(G,B);
                double Min = (min(R,G)<min(G,B))?min(R,G):min(G,B);
                if(Max==Min){Max+=1;}
                if(R==Max){H_sum=(G-B)*60/(Max-Min);}
                if(G==Max){H_sum=120+(B-R)*60/(Max-Min);}
                if(B==Max){H_sum=240+(R-G)*60/(Max-Min);}
                if(B==G&&B==R){H_sum=0;}
                if(H_sum<0){H_sum=H_sum+360;}
                if(Max==0){S_sum=0;}
                S_sum=(((Max-Min)*100)/Max);
                V_sum=Max*100;

                HSVmap[r+(g<<8)+(b<<16)] = 0x00;
                if(BallHSVBoxMsg[0] < BallHSVBoxMsg[1]){
                    if( (H_sum >= BallHSVBoxMsg[0]) && (H_sum <= BallHSVBoxMsg[1])
                      &&(S_sum >= BallHSVBoxMsg[2]) && (S_sum <= BallHSVBoxMsg[3])
                      &&(V_sum >= BallHSVBoxMsg[4]) && (V_sum <= BallHSVBoxMsg[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | REDITEM;

                }else{
                    if( (H_sum >= BallHSVBoxMsg[0]) || (H_sum <= BallHSVBoxMsg[1])
                      &&(S_sum >= BallHSVBoxMsg[2]) && (S_sum <= BallHSVBoxMsg[3])
                      &&(V_sum >= BallHSVBoxMsg[4]) && (V_sum <= BallHSVBoxMsg[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | REDITEM;
                }
                if(GreenHSVBoxMsg[0] < GreenHSVBoxMsg[1]){
                    if( (H_sum >= GreenHSVBoxMsg[0]) && (H_sum <= GreenHSVBoxMsg[1])
                      &&(S_sum >= GreenHSVBoxMsg[2]) && (S_sum <= GreenHSVBoxMsg[3])
                      &&(V_sum >= GreenHSVBoxMsg[4]) && (V_sum <= GreenHSVBoxMsg[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | GREENITEM;
                }else{
                    if( (H_sum >= GreenHSVBoxMsg[0]) || (H_sum <= GreenHSVBoxMsg[1])
                      &&(S_sum >= GreenHSVBoxMsg[2]) && (S_sum <= GreenHSVBoxMsg[3])
                      &&(V_sum >= GreenHSVBoxMsg[4]) && (V_sum <= GreenHSVBoxMsg[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | GREENITEM;
                }
                if(BlueHSVBoxMsg[0] < BlueHSVBoxMsg[1]){
                    if( (H_sum >= BlueHSVBoxMsg[0]) && (H_sum <= BlueHSVBoxMsg[1])
                      &&(S_sum >= BlueHSVBoxMsg[2]) && (S_sum <= BlueHSVBoxMsg[3])
                      &&(V_sum >= BlueHSVBoxMsg[4]) && (V_sum <= BlueHSVBoxMsg[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | BLUEITEM;
                }else{
                    if( (H_sum >= BlueHSVBoxMsg[0]) || (H_sum <= BlueHSVBoxMsg[1])
                      &&(S_sum >= BlueHSVBoxMsg[2]) && (S_sum <= BlueHSVBoxMsg[3])
                      &&(V_sum >= BlueHSVBoxMsg[4]) && (V_sum <= BlueHSVBoxMsg[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | BLUEITEM;
                }
                if(YellowHSVBoxMsg[0] < YellowHSVBoxMsg[1]){
                    if( (H_sum >= YellowHSVBoxMsg[0]) && (H_sum <= YellowHSVBoxMsg[1])
                      &&(S_sum >= YellowHSVBoxMsg[2]) && (S_sum <= YellowHSVBoxMsg[3])
                      &&(V_sum >= YellowHSVBoxMsg[4]) && (V_sum <= YellowHSVBoxMsg[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | YELLOWITEM;
                }else{
                    if( (H_sum >= YellowHSVBoxMsg[0]) || (H_sum <= YellowHSVBoxMsg[1])
                      &&(S_sum >= YellowHSVBoxMsg[2]) && (S_sum <= YellowHSVBoxMsg[3])
                      &&(V_sum >= YellowHSVBoxMsg[4]) && (V_sum <= YellowHSVBoxMsg[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | YELLOWITEM;
                }
                if(HSV_white[0] < HSV_white[1]){
                    if( (H_sum >= HSV_white[0]) && (H_sum <= HSV_white[1])
                      &&(S_sum >= HSV_white[2]) && (S_sum <= HSV_white[3])
                      &&(V_sum >= HSV_white[4]) && (V_sum <= HSV_white[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | WHITEITEM;
                }else{
                    if( (H_sum >= HSV_white[0]) || (H_sum <= HSV_white[1])
                      &&(S_sum >= HSV_white[2]) && (S_sum <= HSV_white[3])
                      &&(V_sum >= HSV_white[4]) && (V_sum <= HSV_white[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | WHITEITEM;
                }
            }
        }
    }

    string Filename = vision_path+FILE_PATH;
    const char *Filename_Path = Filename.c_str();

    if(SaveButton!=0){
    FILE *file=fopen(Filename_Path,"rb+"); //開啟檔案來寫
    fwrite( HSVmap, 1, 256*256*256 , file );
    fclose(file);
    SaveButton = 0;
  }
}
/// ////////////////////////////////////////////////////////////////////
*/











