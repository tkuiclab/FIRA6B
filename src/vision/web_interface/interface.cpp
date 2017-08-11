#define PI 3.14159265
#include "interface.hpp"
#define FRAME_COLS 659 //width  x695
#define FRAME_ROWS 493 //height y493
#define REDITEM 0x01
#define GREENITEM 0x02
#define BLUEITEM 0x04
#define YELLOWITEM 0x08
#define WHITEITEM 0x10
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
  }
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
  Parameter_getting(1);

  image_sub_ = it_.subscribe("/camera/image_raw", 1, &InterfaceProc::imageCb, this);
  //image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &InterfaceProc::imageCb, this);
  image_pub_threshold_ = it_.advertise("/camera/image", 1);//http://localhost:8080/stream?topic=/camera/image webfor /camera/image
  object_pub = nh.advertise<vision::Object>("/vision/object", 1);
  CenterDis_pub = nh.advertise<vision::dis>("/interface/CenterDis", 1);
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
cv::Mat InterfaceProc::ScanModel(const cv::Mat iframe)
{
  static cv::Mat oframe(cv::Size(iframe.cols, iframe.rows), CV_8UC3);
  oframe = iframe;
  int x, y;

  Unscaned_Angle[0] = Angle_Adjustment(Dont_Search_Angle_1Msg - Angle_range_1Msg);
  Unscaned_Angle[1] = Angle_Adjustment(Dont_Search_Angle_1Msg + Angle_range_1Msg);
  Unscaned_Angle[2] = Angle_Adjustment(Dont_Search_Angle_2Msg - Angle_range_2_3Msg);
  Unscaned_Angle[3] = Angle_Adjustment(Dont_Search_Angle_2Msg + Angle_range_2_3Msg);
  Unscaned_Angle[4] = Angle_Adjustment(Dont_Search_Angle_3Msg - Angle_range_2_3Msg);
  Unscaned_Angle[5] = Angle_Adjustment(Dont_Search_Angle_3Msg + Angle_range_2_3Msg);
  Unscaned_Angle[6] = 999;
  Unscaned_Angle[7] = 999;

  if((Dont_Search_Angle_1Msg - Angle_range_1Msg)<=0 || (Dont_Search_Angle_1Msg + Angle_range_1Msg)>=360){
    Unscaned_Angle[0] = 0;
    Unscaned_Angle[1] = Angle_Adjustment(Dont_Search_Angle_1Msg + Angle_range_1Msg);
    Unscaned_Angle[2] = Angle_Adjustment(Dont_Search_Angle_2Msg - Angle_range_2_3Msg);
    Unscaned_Angle[3] = Angle_Adjustment(Dont_Search_Angle_2Msg + Angle_range_2_3Msg);
    Unscaned_Angle[4] = Angle_Adjustment(Dont_Search_Angle_3Msg - Angle_range_2_3Msg);
    Unscaned_Angle[5] = Angle_Adjustment(Dont_Search_Angle_3Msg + Angle_range_2_3Msg);
    Unscaned_Angle[6] = Angle_Adjustment(Dont_Search_Angle_1Msg - Angle_range_1Msg);
    Unscaned_Angle[7] = 360;
  }

  if((Dont_Search_Angle_2Msg - Angle_range_2_3Msg)<=0 || (Dont_Search_Angle_2Msg + Angle_range_2_3Msg)>=360){
    Unscaned_Angle[0] = Angle_Adjustment(Dont_Search_Angle_1Msg - Angle_range_1Msg);
    Unscaned_Angle[1] = Angle_Adjustment(Dont_Search_Angle_1Msg + Angle_range_1Msg);
    Unscaned_Angle[2] = 0;
    Unscaned_Angle[3] = Angle_Adjustment(Dont_Search_Angle_2Msg + Angle_range_2_3Msg);
    Unscaned_Angle[4] = Angle_Adjustment(Dont_Search_Angle_3Msg - Angle_range_2_3Msg);
    Unscaned_Angle[5] = Angle_Adjustment(Dont_Search_Angle_3Msg + Angle_range_2_3Msg);
    Unscaned_Angle[6] = Angle_Adjustment(Dont_Search_Angle_2Msg - Angle_range_2_3Msg);
    Unscaned_Angle[7] = 360;
  }

  if((Dont_Search_Angle_3Msg - Angle_range_2_3Msg)<=0 || (Dont_Search_Angle_3Msg + Angle_range_2_3Msg)>=360){
    Unscaned_Angle[0] = Angle_Adjustment(Dont_Search_Angle_1Msg - Angle_range_1Msg);
    Unscaned_Angle[1] = Angle_Adjustment(Dont_Search_Angle_1Msg + Angle_range_1Msg);
    Unscaned_Angle[2] = Angle_Adjustment(Dont_Search_Angle_2Msg - Angle_range_2_3Msg);
    Unscaned_Angle[3] = Angle_Adjustment(Dont_Search_Angle_2Msg + Angle_range_2_3Msg);
    Unscaned_Angle[4] = 0;
    Unscaned_Angle[5] = Angle_Adjustment(Dont_Search_Angle_3Msg + Angle_range_2_3Msg);
    Unscaned_Angle[6] = Angle_Adjustment(Dont_Search_Angle_3Msg - Angle_range_2_3Msg);
    Unscaned_Angle[7] = 360;
  }

  for (int radius = Magn_Near_StartMsg ; radius <= Magn_Far_EndMsg ; radius += Magn_Near_GapMsg) {
    for (int angle = 0 ; angle < 360 ;) {
      //略過柱子
      if ((angle >= Unscaned_Angle[0] && angle <= Unscaned_Angle[1]) ||
          (angle >= Unscaned_Angle[2] && angle <= Unscaned_Angle[3]) ||
          (angle >= Unscaned_Angle[4] && angle <= Unscaned_Angle[5]) ||
          (angle >= Unscaned_Angle[6] && angle <= Unscaned_Angle[7])) {
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
    double x_ = Angle_cos[angle_be]; //Angle_cos[angle_be];
    double y_ = Angle_sin[angle_be]; //Angle_sin[angle_be];

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

    double x_ = Angle_cos[angle_be]; //Angle_cos[angle_be];
    double y_ = Angle_sin[angle_be]; //Angle_sin[angle_be];
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
