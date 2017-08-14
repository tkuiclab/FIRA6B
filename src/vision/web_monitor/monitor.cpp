#define PI 3.14159265
#include "monitor.hpp"
#define FRAME_COLS 659 //width  x695
#define FRAME_ROWS 493 //height y493
#define REDITEM 0x01
#define GREENITEM 0x02
#define BLUEITEM 0x04
#define YELLOWITEM 0x08
#define WHITEITEM 0x10//WHITEITEM=robot
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

void InterfaceProc::Parameter_getting(const int x)
{
  //cout<<"Read the yaml file"<<endl;
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

  int Angle_Adjustment(int angle);

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

  center_x = CenterXMsg;
  center_y = CenterYMsg;
  center_inner = InnerMsg;
  center_outer = OuterMsg;
  center_front = FrontMsg;
  Camera_H = Camera_HighMsg;

  nh.getParam("/FIRA/Parameterbutton", buttonmsg);
}
void InterfaceProc::SaveButton_setting(const vision::bin msg)
{
  SaveButton = msg.bin;
  Parameter_getting(1);
  HSVmap();
}
void InterfaceProc::View(const vision::view msg)
{
  viewcheck = msg.checkpoint;
}
InterfaceProc::InterfaceProc()
  : it_(nh)
{
  ros::NodeHandle n("~");
  Parameter_getting(1);
  frame_counter = 0;

  double ang_PI;
  for (int ang = 0 ; ang < 360; ang++) {
    ang_PI = ang * PI / 180;
    Angle_sin.push_back(sin(ang_PI));
    Angle_cos.push_back(cos(ang_PI));
  }

  image_sub_ = it_.subscribe("/camera/image_raw", 1, &InterfaceProc::imageCb, this);
  //image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &InterfaceProc::imageCb, this);
  image_pub_threshold_ = it_.advertise("/camera/image_monitor", 1);//http://localhost:8080/stream?topic=/camera/image_monitor webfor /camera/image

  s1 = nh.subscribe("interface/bin_save", 1000, &InterfaceProc::SaveButton_setting, this);
  s2 = nh.subscribe("vision/view", 1000, &InterfaceProc::View, this);
  object_pub = nh.advertise<vision::Object>("/vision/object", 1);
  Two_point_pub = nh.advertise<vision::Two_point>("/interface/Two_point", 1);
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
//////////////////////////////////////////////////////////////
  vision_path = ros::package::getPath("vision");
  color_map = ColorFile();

  object_Item_reset(Red_Item);
  object_Item_reset(Blue_Item);
  object_Item_reset(Yellow_Item);

  objectdet_change(Findmap, REDITEM, Red_Item);
  objectdet_change(Findmap, BLUEITEM, Blue_Item);
  objectdet_change(Findmap, YELLOWITEM, Yellow_Item);

  vision::Object object_msg;

  int Angle_Adjustment(int angle);
  
  if (Red_Item.distance != 0) {
    object_msg.ball_x = Red_Item.x - CenterXMsg;
    object_msg.ball_y = 0 - (Red_Item.y - CenterYMsg);
    object_msg.ball_LR = Red_Item.LR;
    object_msg.ball_ang = Strategy_Angle(Angle_Adjustment(Red_Item.angle));
    object_msg.ball_dis = Omni_distance(Red_Item.distance);
    object_msg.goalkeeper_move = Red_Item.gkm;
    object_msg.ball_fly = Red_Item.fly; 
  } else {
    object_msg.ball_ang = 999;
    object_msg.ball_dis = 999;
    object_msg.ball_LR = "Null";
  }

  if (Blue_Item.distance != 0) {
    object_msg.blue_x = Blue_Item.x - CenterXMsg;
    object_msg.blue_y = 0 - (Blue_Item.y - CenterYMsg);
    object_msg.blue_LR = Blue_Item.LR;
    object_msg.blue_ang = Strategy_Angle(Angle_Adjustment(Blue_Item.angle));
    object_msg.blue_dis = Omni_distance(Blue_Item.distance);
    object_msg.blue_fix_x = Blue_Item.fix_x - CenterXMsg;
    object_msg.blue_fix_y =  0 - (Blue_Item.fix_y - CenterYMsg);
    object_msg.blue_fix_ang = Strategy_Angle(Angle_Adjustment(Blue_Item.fix_angle));
    object_msg.blue_fix_dis = Omni_distance(Blue_Item.fix_distance); 
  } else {
    object_msg.blue_ang = 999;
    object_msg.blue_dis = 999;
    object_msg.blue_LR = "Null";
    object_msg.blue_fix_ang = 999;
    object_msg.blue_fix_dis = 999;
  }

  if (Yellow_Item.distance != 0) {
    object_msg.yellow_x = Yellow_Item.x - CenterXMsg;
    object_msg.yellow_y = 0 - (Yellow_Item.y - CenterYMsg);
    object_msg.yellow_LR = Yellow_Item.LR;
    object_msg.yellow_ang = Strategy_Angle(Angle_Adjustment(Yellow_Item.angle));
    object_msg.yellow_dis = Omni_distance(Yellow_Item.distance);
    object_msg.yellow_fix_x = Yellow_Item.fix_x - CenterXMsg;
    object_msg.yellow_fix_y = 0 - (Yellow_Item.fix_y - CenterYMsg);
    object_msg.yellow_fix_ang = Strategy_Angle(Angle_Adjustment(Yellow_Item.fix_angle));
    object_msg.yellow_fix_dis = Omni_distance(Yellow_Item.fix_distance);
  } else {
    object_msg.yellow_ang = 999;
    object_msg.yellow_dis = 999;
    object_msg.yellow_LR = "Null";
    object_msg.yellow_fix_ang = 999;
    object_msg.yellow_fix_dis = 999;
  }
////////////////////////////////////////////////////////////////////////////
  vision::Two_point Two_point_msg;

  if(Blue_Item.distance != 0){
    Two_point_msg.blue_dis = Omni_distance(Blue_Item.distance);
    Two_point_msg.blue_ang_max = Strategy_Angle(Angle_Adjustment(Blue_Item.ang_max));
    Two_point_msg.blue_ang_min = Strategy_Angle(Angle_Adjustment(Blue_Item.ang_min));
    Two_point_msg.blue_fix_ang_max = Strategy_Angle(Angle_Adjustment(Blue_Item.fix_ang_max));
    Two_point_msg.blue_fix_ang_min = Strategy_Angle(Angle_Adjustment(Blue_Item.fix_ang_min));   
    Two_point_msg.blue_left = Omni_distance(Blue_Item.left_dis);
    Two_point_msg.blue_right = Omni_distance(Blue_Item.right_dis);
  } else {
    Two_point_msg.blue_dis = 999;
  }
  if(Yellow_Item.distance != 0){
    Two_point_msg.yellow_dis = Omni_distance(Yellow_Item.distance);
    Two_point_msg.yellow_ang_max = Strategy_Angle(Angle_Adjustment(Yellow_Item.ang_max));
    Two_point_msg.yellow_ang_min = Strategy_Angle(Angle_Adjustment(Yellow_Item.ang_min));
    Two_point_msg.yellow_fix_ang_max = Strategy_Angle(Angle_Adjustment(Yellow_Item.fix_ang_max));
    Two_point_msg.yellow_fix_ang_min = Strategy_Angle(Angle_Adjustment(Yellow_Item.fix_ang_min));
    Two_point_msg.yellow_left = Omni_distance(Yellow_Item.left_dis);
    Two_point_msg.yellow_right = Omni_distance(Yellow_Item.right_dis);
  } else {
    Two_point_msg.yellow_dis = 999;
  }
/////////////////////FPS///////////////////////
  frame_counter++;
  //static long int StartTime = time(NULL);//ros::Time::now().toNSec();
  static long int StartTime = ros::Time::now().toNSec();
  //static long int EndTime;
  static long double FrameRate = 0.0;

  if (frame_counter == 10) {
    EndTime = ros::Time::now().toNSec();
    dt = (EndTime - StartTime) / frame_counter;
    StartTime = EndTime;
    if ( dt != 0 )
    {
      FrameRate = ( 1000000000.0 / dt ) * ALPHA + FrameRate * ( 1.0 - ALPHA );
      //cout << "FPS: " << FrameRate << endl;
    }
    frame_counter = 0;
  }
  object_msg.fps = FrameRate;
///////////////////////////////////////////////
  Findmap.release();

  object_pub.publish(object_msg);
  Two_point_pub.publish(Two_point_msg);

  sensor_msgs::ImagePtr thresholdMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Main_frame).toImageMsg();
  if (viewcheck == 64) {
    image_pub_threshold_.publish(thresholdMsg);
  }

}
//////////////////////處理影像開始//////////////////////////////////////
//////////////////////////////monitor//////////////////////////////////
void InterfaceProc::object_Item_reset(object_Item &obj_) {
  obj_.dis_max = 0;
  obj_.dis_min = 0;
  obj_.ang_max = 0;
  obj_.ang_min = 0;
  obj_.x = 0;
  obj_.y = 0;
  obj_.angle = 0;
  obj_.distance = 0;
  obj_.size = 0;
  obj_.LR = "Null";

  obj_.left_dis = 999;
  obj_.right_dis = 999;
  obj_.left_x = 0;
  obj_.left_y = 0;
  obj_.right_x = 0;
  obj_.right_y = 0;
  obj_.fix_x = 0;
  obj_.fix_y = 0;
  obj_.fix_angle = 0;
  obj_.fix_distance = 0;
  obj_.fix_ang_max = 0;
  obj_.fix_ang_min = 0;

  obj_.gkm = 0;
  obj_.fly = 0;
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
//right -0~-180
//left   0~180
int InterfaceProc::Strategy_Angle(int angle)
{
  if (Angle_Adjustment(angle - center_front) < 180) {
    angle = Angle_Adjustment(angle - center_front);
  } else {
    angle = Angle_Adjustment(angle - center_front) - 360;
  }
  return angle;
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
void InterfaceProc::objectdet_change(Mat &frame_, int color, object_Item &obj_item) {
  int x, y;
  int x_, y_;
  int object_size;
  int dis, ang;

  frame_ = Mat(Size(Main_frame.cols, Main_frame.rows), CV_8UC3, Scalar(0, 0, 0));

  find_point.clear();
  object_Item_reset(FIND_Item);

  for (int distance = search_start ; distance <= search_end ; distance += search_distance) {
    for (int angle = 0; angle < 360;) {
      if ((angle >= Unscaned_Angle[0] && angle <= Unscaned_Angle[1]) ||
          (angle >= Unscaned_Angle[2] && angle <= Unscaned_Angle[3]) ||
          (angle >= Unscaned_Angle[4] && angle <= Unscaned_Angle[5]) ||
          (angle >= Unscaned_Angle[6] && angle <= Unscaned_Angle[7])) {
        angle += Angle_Interval(distance);
        continue;
      }
      object_size = 0;
      FIND_Item.size = 0;

      x_ = distance * Angle_cos[angle];
      y_ = distance * Angle_sin[angle];

      x = Frame_Area(center_x + x_, frame_.cols);
      y = Frame_Area(center_y - y_, frame_.rows);

      unsigned char B = Main_frame.data[(y * Main_frame.cols + x) * 3 + 0];
      unsigned char G = Main_frame.data[(y * Main_frame.cols + x) * 3 + 1];
      unsigned char R = Main_frame.data[(y * Main_frame.cols + x) * 3 + 2];
      if (color_map[R + (G << 8) + (B << 16)] & color && frame_.data[(y * frame_.cols + x) * 3 + 0] == 0) {
        Mark_point(frame_, distance, angle , x , y, object_size, color);
        FIND_Item.dis_max = distance;
        FIND_Item.dis_min = distance;
        FIND_Item.ang_max = angle;
        FIND_Item.ang_min = angle;
        while (!find_point.empty()) {
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

      if (FIND_Item.size > obj_item.size) {
        obj_item = FIND_Item;
      }

      angle += Angle_Interval(distance);
    }
  }

  find_object_point(obj_item, color);

  draw_ellipse(Main_frame, Red_Item, REDITEM);
  draw_ellipse(Main_frame, Yellow_Item, YELLOWITEM);
  draw_ellipse(Main_frame, Blue_Item, BLUEITEM);

  if (Red_Item.x != 0) {Draw_cross(Main_frame, 'R');}
  if (Blue_Item.x != 0) {Draw_cross(Main_frame, 'B');}
  if (Yellow_Item.x != 0) {Draw_cross(Main_frame, 'Y');}
}
void InterfaceProc::Mark_point(Mat &frame_, int distance, int angle, int x, int y, int &size, int color) {
  frame_.data[(y * frame_.cols + x) * 3 + 0] = 255;
  frame_.data[(y * frame_.cols + x) * 3 + 1] = 255;
  frame_.data[(y * frame_.cols + x) * 3 + 2] = 255;
  find_point.push_back(distance);
  find_point.push_back(angle);
  size += 1;
}
void InterfaceProc::find_around(Mat &frame_, int distance , int angle, int &size, int color) {
  int x, y;
  int x_, y_;
  int dis_f, ang_f;
  double angle_f;

  for (int i = -1 ; i < 2 ; i++) {
    for (int j = -1 ; j < 2 ; j++) {
      dis_f = distance + i * search_distance;

      if (dis_f < search_start) dis_f = search_start;

      if (color == REDITEM || color == BLUEITEM || color == YELLOWITEM) {
        dis_f = Frame_Area(dis_f, search_end);
      } 

      ang_f = angle + j * Angle_Interval(dis_f);

      while ((Angle_Adjustment(ang_f) > Unscaned_Angle[0] && Angle_Adjustment(ang_f) < Unscaned_Angle[1]) ||
             (Angle_Adjustment(ang_f) > Unscaned_Angle[2] && Angle_Adjustment(ang_f) < Unscaned_Angle[3]) ||
             (Angle_Adjustment(ang_f) > Unscaned_Angle[4] && Angle_Adjustment(ang_f) < Unscaned_Angle[5]) || 
             (Angle_Adjustment(ang_f) > Unscaned_Angle[6] && Angle_Adjustment(ang_f) < Unscaned_Angle[7])) {
        ang_f += j * Angle_Interval(dis_f);
      }

      angle_f = Angle_Adjustment(ang_f);

      x_ = dis_f * Angle_cos[angle_f];
      y_ = dis_f * Angle_sin[angle_f];

      x = Frame_Area(center_x + x_, frame_.cols);
      y = Frame_Area(center_y - y_, frame_.rows);

      unsigned char B = Main_frame.data[(y * Main_frame.cols + x) * 3 + 0];
      unsigned char G = Main_frame.data[(y * Main_frame.cols + x) * 3 + 1];
      unsigned char R = Main_frame.data[(y * Main_frame.cols + x) * 3 + 2];

      if (color == REDITEM || color == BLUEITEM || color == YELLOWITEM) {
        if (color_map[R + (G << 8) + (B << 16)] & color && frame_.data[(y * frame_.cols + x) * 3 + 0] == 0) {
          Mark_point(frame_, dis_f, ang_f , x , y, size, color);
        }
      } 
    }
  }
}
void InterfaceProc::find_object_point(object_Item &obj_, int color) {
  int x=0, y=0;
  int x_, y_;
  int angle_;
  int angle_range;
  int find_angle;
  unsigned char B, G, R;
  //object center point
  if (color == REDITEM){// || color ==  BLUEITEM || color == YELLOWITEM) {
    angle_ = Angle_Adjustment((obj_.ang_max + obj_.ang_min) / 2);
    angle_range = 0.7 * Angle_Adjustment((obj_.ang_max - obj_.ang_min) / 2);
    if(color == REDITEM && obj_.ang_max - obj_.ang_min >= 2) angle_range = (obj_.ang_max - obj_.ang_min) / 2;
    for (int angle = 0 ; angle < angle_range ; angle++) {
      for (int distance = obj_.dis_min ; distance <= (obj_.dis_min + obj_.dis_max) / 2 ; distance++) {
        if (obj_.distance != 0) break; 
        
        find_angle = Angle_Adjustment(angle_ + angle);

        if ((find_angle >= Unscaned_Angle[0] && find_angle <= Unscaned_Angle[1]) ||
            (find_angle >= Unscaned_Angle[2] && find_angle <= Unscaned_Angle[3]) ||
            (find_angle >= Unscaned_Angle[4] && find_angle <= Unscaned_Angle[5]) ||
            (find_angle >= Unscaned_Angle[6] && find_angle <= Unscaned_Angle[7])) {
        } else {
          x_ = distance * Angle_cos[find_angle];
          y_ = distance * Angle_sin[find_angle];

          x = Frame_Area(center_x + x_, Main_frame.cols);
          y = Frame_Area(center_y - y_, Main_frame.rows);

          B = Main_frame.data[(y * Main_frame.cols + x) * 3 + 0];
          G = Main_frame.data[(y * Main_frame.cols + x) * 3 + 1];
          R = Main_frame.data[(y * Main_frame.cols + x) * 3 + 2];

          if (color_map[R + (G << 8) + (B << 16)] & color) {
            obj_.x = x;
            obj_.y = y;
            obj_.distance = distance;
            obj_.angle = find_angle;
            break;
          }
        }

        find_angle = Angle_Adjustment(angle_ - angle);
        if ((find_angle >= Unscaned_Angle[0] && find_angle <= Unscaned_Angle[1]) ||
            (find_angle >= Unscaned_Angle[2] && find_angle <= Unscaned_Angle[3]) ||
            (find_angle >= Unscaned_Angle[4] && find_angle <= Unscaned_Angle[5]) ||
            (find_angle >= Unscaned_Angle[6] && find_angle <= Unscaned_Angle[7])) {
        } else {
          x_ = distance * Angle_cos[find_angle];
          y_ = distance * Angle_sin[find_angle];

          x = Frame_Area(center_x + x_, Main_frame.cols);
          y = Frame_Area(center_y - y_, Main_frame.rows);

          B = Main_frame.data[(y * Main_frame.cols + x) * 3 + 0];
          G = Main_frame.data[(y * Main_frame.cols + x) * 3 + 1];
          R = Main_frame.data[(y * Main_frame.cols + x) * 3 + 2];

          if (color_map[R + (G << 8) + (B << 16)] & color) {
            obj_.x = x;
            obj_.y = y;
            obj_.distance = distance;
            obj_.angle = find_angle;
            break;
          }
        }
      }
    }
  }
  if (color ==  BLUEITEM || color == YELLOWITEM) {
    if(obj_.ang_max - obj_.ang_min > 4){
      find_angle = Angle_Adjustment((obj_.ang_max + obj_.ang_min) / 2);
      int distance = obj_.dis_min + 5;
      x_ = distance * Angle_cos[find_angle];
      y_ = distance * Angle_sin[find_angle];

      x = Frame_Area(center_x + x_, Main_frame.cols);
      y = Frame_Area(center_y - y_, Main_frame.rows);
      obj_.x = x;
      obj_.y = y;
      obj_.distance = distance;
      obj_.angle = find_angle;
    }
  }
  if(color == REDITEM){
    if(Omni_distance(obj_.distance) < 300 || obj_.dis_max - obj_.dis_min >= 13) obj_.gkm = 1;
    if(Omni_distance(obj_.distance) > 200 && obj_.dis_max - obj_.dis_min >= 18) obj_.fly = 1;
  }
  if(color == YELLOWITEM || color == BLUEITEM){
    if(Omni_distance(obj_.distance) < 300  && obj_.dis_max - obj_.dis_min <= 50){
      obj_.x = 0;
      obj_.y = 0;
      obj_.distance = 0;
      obj_.angle = 0;
    }
  }
  if (Angle_Adjustment(angle_ - center_front) < 180) {
    obj_.LR = "Left";
  } else {
    obj_.LR = "Right";
  }
//找球門邊界點
  if (color ==  BLUEITEM || color == YELLOWITEM) {
    int right_angle, left_angle;
    int temp = 999;
    right_angle = obj_.ang_max;
    left_angle = obj_.ang_min;
  
    //left
    for (int angle = 0 ; angle < 5 ; angle++) {
      for (int distance = obj_.dis_min ; distance <= obj_.dis_max ; distance++) {
        find_angle = Angle_Adjustment(left_angle + angle);
        x_ = distance * Angle_cos[find_angle];
        y_ = distance * Angle_sin[find_angle];

        x = Frame_Area(center_x + x_, Main_frame.cols);
        y = Frame_Area(center_y - y_, Main_frame.rows);

        B = Main_frame.data[(y * Main_frame.cols + x) * 3 + 0];
        G = Main_frame.data[(y * Main_frame.cols + x) * 3 + 1];
        R = Main_frame.data[(y * Main_frame.cols + x) * 3 + 2];

        if (color_map[R + (G << 8) + (B << 16)] & color) {
          temp = distance;
          if(obj_.left_dis-7 > temp){
            obj_.left_x = x;
            obj_.left_y = y;
            obj_.left_dis = temp; 
          }     
        }
      }
    }
    //right
    for (int angle = 0 ; angle < 5 ; angle++) {
      for (int distance = obj_.dis_min ; distance <= obj_.dis_max ; distance++) {
        find_angle = Angle_Adjustment(right_angle - angle);
        x_ = distance * Angle_cos[find_angle];
        y_ = distance * Angle_sin[find_angle];

        x = Frame_Area(center_x + x_, Main_frame.cols);
        y = Frame_Area(center_y - y_, Main_frame.rows);

        B = Main_frame.data[(y * Main_frame.cols + x) * 3 + 0];
        G = Main_frame.data[(y * Main_frame.cols + x) * 3 + 1];
        R = Main_frame.data[(y * Main_frame.cols + x) * 3 + 2];

        if (color_map[R + (G << 8) + (B << 16)] & color) {
          temp = distance;
          if(obj_.right_dis-7 > temp){
            obj_.right_x = x;
            obj_.right_y = y;
            obj_.right_dis = temp;
          }
        }
      }
    }
  }
////////////////////////////////中心點：被障礙物阻擋時偏移修正///////////////////////////////////////////
  if(color == BLUEITEM || color == YELLOWITEM){
		//找最大範圍
		int find_gap[2][7]={0};
		int start = obj_.dis_min;

		if(obj_.dis_min > 130){start = obj_.dis_min - 20;}
		for (int angle =  obj_.ang_min ; angle <= obj_.ang_max ; angle++) {
			for (int distance = start ; distance <= (obj_.dis_min + obj_.dis_max) / 2 ; distance++) {
				find_angle = Angle_Adjustment(angle);
   
				if ((find_angle >= Unscaned_Angle[0] && find_angle <= Unscaned_Angle[1]) ||
				    (find_angle >= Unscaned_Angle[2] && find_angle <= Unscaned_Angle[3]) ||
				    (find_angle >= Unscaned_Angle[4] && find_angle <= Unscaned_Angle[5]) ||
				    (find_angle >= Unscaned_Angle[6] && find_angle <= Unscaned_Angle[7])) {
					if(angle!=obj_.ang_max)break;
				}
				//中心座標
				x_ = distance * Angle_cos[find_angle];
				y_ = distance * Angle_sin[find_angle];
				//實際座標
				x = Frame_Area(center_x + x_, Main_frame.cols);
				y = Frame_Area(center_y - y_, Main_frame.rows);

				B = Main_frame.data[(y * Main_frame.cols + x) * 3 + 0];
				G = Main_frame.data[(y * Main_frame.cols + x) * 3 + 1];
				R = Main_frame.data[(y * Main_frame.cols + x) * 3 + 2];

				if(color_map[R + (G << 8) + (B << 16)] & WHITEITEM || angle == obj_.ang_max){
					find_gap[1][6] = find_gap[1][5] - find_gap[1][2];
					if(find_gap[0][6] < find_gap[1][6]){
						if(color == BLUEITEM){
						if(b_end_gap !=0 && 
							find_gap[1][6] < ((obj_.ang_max - obj_.ang_min) * 0.4) && 
							(abs(find_gap[1][5] + find_gap[1][2]) / 2 - b_end_gap) > ((obj_.ang_max - obj_.ang_min) * 0.3)){
							} else {
								for(int i=0;i<7;i++){
									find_gap[0][i] = find_gap[1][i];
								}
							} 
						}
						if(color == YELLOWITEM){
							if(y_end_gap !=0 && 
							find_gap[1][6] < ((obj_.ang_max - obj_.ang_min) * 0.4) && 
							(abs(find_gap[1][5] + find_gap[1][2]) / 2 - y_end_gap) > ((obj_.ang_max - obj_.ang_min) * 0.3)){
							} else {
								for(int i=0;i<7;i++){
									find_gap[0][i] = find_gap[1][i];
								}
							} 
						}
					}
					find_gap[1][0] = 0;
					find_gap[1][1] = 0;
					find_gap[1][2] = 0;
					find_gap[1][3] = 0;
					find_gap[1][4] = 0;
					find_gap[1][5] = 0;
					find_gap[1][6] = 0;
					break;
				}

				if (color_map[R + (G << 8) + (B << 16)] & color) {
					if(find_gap[1][0] == 0){
						find_gap[1][0] = x;
						find_gap[1][1] = y;
						find_gap[1][2] = angle;
					}else{
						find_gap[1][3] = x;
						find_gap[1][4] = y;
						find_gap[1][5] = angle;
					}
				}
			}
		}	
		obj_.fix_ang_min = find_gap[0][2];
		obj_.fix_ang_max = find_gap[0][5]; 
		if(color == BLUEITEM){
			if(find_gap[0][5] > 0) b_end_gap = (find_gap[0][5] + find_gap[0][2]) / 2;
			else b_end_gap = 0;
		}
		if(color == YELLOWITEM){
			if(find_gap[0][5] > 0) y_end_gap = (find_gap[0][5] + find_gap[0][2]) / 2;
			else y_end_gap = 0;
		}

    //找中心
    angle_ = Angle_Adjustment((find_gap[0][5] + find_gap[0][2]) / 2);
    angle_range = 0.7 * Angle_Adjustment((find_gap[0][5] - find_gap[0][2]) / 2);
    for (int angle = 0 ; angle < angle_range ; angle++) {
      for (int distance = obj_.dis_min ; distance <=  (obj_.dis_min + obj_.dis_max) / 2 ; distance++) {
        if (obj_.fix_distance != 0) break;
        find_angle = Angle_Adjustment(angle_ + angle);
        if ((find_angle >= Unscaned_Angle[0] && find_angle <= Unscaned_Angle[1]) ||
            (find_angle >= Unscaned_Angle[2] && find_angle <= Unscaned_Angle[3]) ||
            (find_angle >= Unscaned_Angle[4] && find_angle <= Unscaned_Angle[5]) ||
            (find_angle >= Unscaned_Angle[6] && find_angle <= Unscaned_Angle[7])) {
        } else {
          x_ = distance * Angle_cos[find_angle];
          y_ = distance * Angle_sin[find_angle];

          x = Frame_Area(center_x + x_, Main_frame.cols);
          y = Frame_Area(center_y - y_, Main_frame.rows);

          B = Main_frame.data[(y * Main_frame.cols + x) * 3 + 0];
          G = Main_frame.data[(y * Main_frame.cols + x) * 3 + 1];
          R = Main_frame.data[(y * Main_frame.cols + x) * 3 + 2];

          if (color_map[R + (G << 8) + (B << 16)] & color) {
            obj_.fix_x = x;
            obj_.fix_y = y;
            obj_.fix_distance = distance;
            obj_.fix_angle = find_angle;
            break;
          }
        }
        find_angle = Angle_Adjustment(angle_ - angle);
        if ((find_angle >= Unscaned_Angle[0] && find_angle <= Unscaned_Angle[1]) ||
            (find_angle >= Unscaned_Angle[2] && find_angle <= Unscaned_Angle[3]) ||
            (find_angle >= Unscaned_Angle[4] && find_angle <= Unscaned_Angle[5]) ||
            (find_angle >= Unscaned_Angle[6] && find_angle <= Unscaned_Angle[7])) {
        } else {
          x_ = distance * Angle_cos[find_angle];
          y_ = distance * Angle_sin[find_angle];

          x = Frame_Area(center_x + x_, Main_frame.cols);
          y = Frame_Area(center_y - y_, Main_frame.rows);

          B = Main_frame.data[(y * Main_frame.cols + x) * 3 + 0];
          G = Main_frame.data[(y * Main_frame.cols + x) * 3 + 1];
          R = Main_frame.data[(y * Main_frame.cols + x) * 3 + 2];

          if (color_map[R + (G << 8) + (B << 16)] & color) {
            obj_.fix_x = x;
            obj_.fix_y = y;
            obj_.fix_distance = distance;
            obj_.fix_angle = find_angle;
            break;
          }
        }
      }
    }

    if(obj_.fix_distance == 0){
      obj_.fix_x = obj_.x;
      obj_.fix_y = obj_.y;
      obj_.fix_angle = obj_.angle;
      obj_.fix_distance = obj_.distance;
      obj_.fix_ang_min = obj_.ang_min;
      obj_.fix_ang_max = obj_.ang_max; 
    }
  }
}
void InterfaceProc::draw_ellipse(Mat &frame_, object_Item &obj_, int color) {
  ellipse(frame_, Point(center_x, center_y), Size(obj_.dis_min, obj_.dis_min), 0, 360 - obj_.ang_max, 360 - obj_.ang_min, Scalar(255, 255, 0), 1);
  ellipse(frame_, Point(center_x, center_y), Size(obj_.dis_max, obj_.dis_max), 0, 360 - obj_.ang_max, 360 - obj_.ang_min, Scalar(255, 255, 0), 1);
  draw_Line(frame_, obj_.dis_max, obj_.dis_min, obj_.ang_max);
  draw_Line(frame_, obj_.dis_max, obj_.dis_min, obj_.ang_min);
  circle(frame_, Point(obj_.x, obj_.y), 2, Scalar(0, 0, 0), -1);

////////////////////////////center//////////////////////////
  int lengh = 30;
  int x, y;
  //cout<<CenterXMsg;
  if (0 < CenterXMsg < 600) {} else {CenterXMsg = 0; CenterYMsg = 0; InnerMsg = 0; OuterMsg = 0; FrontMsg = 0;} //avoid code dump

  robotCenterX = CenterXMsg; //iframe.cols*(CenterXMsg*1);
  robotCenterY = CenterYMsg; //iframe.rows*(CenterYMsg*1);

  circle(frame_, Point(robotCenterX, robotCenterY), 1, Scalar(0, 255, 0), 1);
  circle(frame_, Point(robotCenterX, robotCenterY), InnerMsg , Scalar(0, 0, 255), 1);
  circle(frame_, Point(robotCenterX, robotCenterY), OuterMsg , Scalar(0, 255, 0), 1);
  x = robotCenterX + lengh * cos(FrontMsg * PI / 180), y = robotCenterY - lengh * sin(FrontMsg * PI / 180);
  line(frame_, Point(robotCenterX, robotCenterY), Point(x, y), Scalar(255, 0, 255), 1);
////////////////////////////////////////////////////////////

  if(color == BLUEITEM || color == YELLOWITEM){
    if(obj_.distance != 0){
   
      x = obj_.right_x;
      y = obj_.right_y;
      line(frame_, Point(x, y), Point(x, y), Scalar(255, 0, 0), 3);

      x = obj_.left_x;
      y = obj_.left_y;
      line(frame_, Point(x, y), Point(x, y), Scalar(0, 0, 255), 3);

      //attack point
      x = obj_.fix_x;
      y = obj_.fix_y;
      line(frame_, Point(x, y), Point(x, y), Scalar(0, 255, 0), 10);
/////////////////////////////////////////////////////////////////////////
	int x_,y_;
	unsigned char B, G, R;
	if( color == BLUEITEM ||YELLOWITEM){
		int start = obj_.dis_min;
		if(obj_.dis_min > 130){start = obj_.dis_min - ((obj_.dis_max - obj_.dis_min) * 0.3);}
		for (int angle =  obj_.ang_min ; angle <= obj_.ang_max ; angle++) {
			for (int distance = start; distance <= (start + obj_.dis_max)/2; distance++) {
				int find_angle = Angle_Adjustment(angle);
				if ((find_angle >= Unscaned_Angle[0] && find_angle <= Unscaned_Angle[1]) ||
				(find_angle >= Unscaned_Angle[2] && find_angle <= Unscaned_Angle[3]) ||
				(find_angle >= Unscaned_Angle[4] && find_angle <= Unscaned_Angle[5]) ||
				(find_angle >= Unscaned_Angle[6] && find_angle <= Unscaned_Angle[7])) {
					break;
				}    
				//中心座標
				x_ = distance * Angle_cos[find_angle];
				y_ = distance * Angle_sin[find_angle];
				//實際座標
				x = Frame_Area(center_x + x_, Main_frame.cols);
				y = Frame_Area(center_y - y_, Main_frame.rows);

				B = Main_frame.data[(y * Main_frame.cols + x) * 3 + 0];
				G = Main_frame.data[(y * Main_frame.cols + x) * 3 + 1];
				R = Main_frame.data[(y * Main_frame.cols + x) * 3 + 2];
				if(color_map[R + (G << 8) + (B << 16)] & WHITEITEM){
					 line(frame_, Point(x, y), Point(x, y), Scalar(255, 255, 255), 3);
		                         break;
				}
			}
		}	
	}
//////////////////////////////////////////////////
    }
  }
}
void InterfaceProc::draw_Line(Mat &frame_, int obj_distance_max, int obj_distance_min, int obj_angle) {
  int x_, y_;
  double angle_f;
  int x[2], y[2];

  angle_f = Angle_Adjustment(obj_angle);

  x_ = obj_distance_min * Angle_cos[angle_f];
  y_ = obj_distance_min * Angle_sin[angle_f];

  x[0] = Frame_Area(center_x + x_, frame_.cols);
  y[0] = Frame_Area(center_y - y_, frame_.rows);

  x_ = obj_distance_max * Angle_cos[angle_f];
  y_ = obj_distance_max * Angle_sin[angle_f];

  x[1] = Frame_Area(center_x + x_, frame_.cols);
  y[1] = Frame_Area(center_y - y_, frame_.rows);

  line(frame_, Point(x[0], y[0]), Point(x[1], y[1]), Scalar(255, 255, 0), 1);
}
void InterfaceProc::Draw_cross(cv::Mat &frame_, char color) {
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

  switch (color) {
  case 'R':
    for (int i = -2; i <= 2; i++) {
      frame_.data[((Red_Item.y + i)*frame_.cols * 3) + ((Red_Item.x + 0) * 3) + 0] = 0;
      frame_.data[((Red_Item.y + i)*frame_.cols * 3) + ((Red_Item.x + 0) * 3) + 1] = 255;
      frame_.data[((Red_Item.y + i)*frame_.cols * 3) + ((Red_Item.x + 0) * 3) + 2] = 0;
    }
    for (int j = -2; j <= 2; j++) {
      frame_.data[((Red_Item.y + 0)*frame_.cols * 3) + ((Red_Item.x + j) * 3) + 0] = 0;
      frame_.data[((Red_Item.y + 0)*frame_.cols * 3) + ((Red_Item.x + j) * 3) + 1] = 255;
      frame_.data[((Red_Item.y + 0)*frame_.cols * 3) + ((Red_Item.x + j) * 3) + 2] = 0;
    }
    R_X_out << Red_Item.x - CenterXMsg;
    R_Y_out << 0 - (Red_Item.y - CenterYMsg);
    R_X = R_X_out.str();
    R_Y = R_Y_out.str();
    cv::putText(frame_, "R(" + R_X + "," + R_Y + ")", Point(Red_Item.x, Red_Item.y), 0, 0.5, Scalar(0, 0, 255), 1);
    break;
  case 'B':
    for (int i = -2; i <= 2; i++) {
      frame_.data[((Blue_Item.y + i)*frame_.cols * 3) + ((Blue_Item.x + 0) * 3) + 0] = 0;
      frame_.data[((Blue_Item.y + i)*frame_.cols * 3) + ((Blue_Item.x + 0) * 3) + 1] = 0;
      frame_.data[((Blue_Item.y + i)*frame_.cols * 3) + ((Blue_Item.x + 0) * 3) + 2] = 255;
    }
    for (int j = -2; j <= 2; j++) {
      frame_.data[((Blue_Item.y + 0)*frame_.cols * 3) + ((Blue_Item.x + j) * 3) + 0] = 0;
      frame_.data[((Blue_Item.y + 0)*frame_.cols * 3) + ((Blue_Item.x + j) * 3) + 1] = 0;
      frame_.data[((Blue_Item.y + 0)*frame_.cols * 3) + ((Blue_Item.x + j) * 3) + 2] = 255;
    }
    B_X_out << Blue_Item.x - CenterXMsg;
    B_Y_out << 0 - (Blue_Item.y - CenterYMsg);
    B_X = B_X_out.str();
    B_Y = B_Y_out.str();
    cv::putText(frame_, "B(" + B_X + "," + B_Y + ")", Point(Blue_Item.x, Blue_Item.y), 0, 0.5, Scalar(255, 0, 0), 1);
    break;
  case 'Y':
    for (int i = -2; i <= 2; i++) {
      frame_.data[((Yellow_Item.y + i)*frame_.cols * 3) + ((Yellow_Item.x + 0) * 3) + 0] = 255;
      frame_.data[((Yellow_Item.y + i)*frame_.cols * 3) + ((Yellow_Item.x + 0) * 3) + 1] = 0;
      frame_.data[((Yellow_Item.y + i)*frame_.cols * 3) + ((Yellow_Item.x + 0) * 3) + 2] = 0;
    }
    for (int j = -2; j <= 2; j++) {
      frame_.data[((Yellow_Item.y + 0)*frame_.cols * 3) + ((Yellow_Item.x + j) * 3) + 0] = 255;
      frame_.data[((Yellow_Item.y + 0)*frame_.cols * 3) + ((Yellow_Item.x + j) * 3) + 1] = 0;
      frame_.data[((Yellow_Item.y + 0)*frame_.cols * 3) + ((Yellow_Item.x + j) * 3) + 2] = 0;
    }
    Y_X_out << Yellow_Item.x - CenterXMsg;
    Y_Y_out << 0 - (Yellow_Item.y - CenterYMsg);
    Y_X = Y_X_out.str();
    Y_Y = Y_Y_out.str();
    cv::putText(frame_, "Y(" + Y_X + ","+Y_Y+ ")", Point(Yellow_Item.x, Yellow_Item.y), 0, 0.5, Scalar(0, 255, 255), 1);
    break;
  }
}
/////////////////////////////////HSVmap////////////////////////////////
void InterfaceProc::HSVmap()
{
  unsigned char *HSVmap = new unsigned char[256 * 256 * 256];
  nh.getParam("/FIRA/HSV/Ball", HSV_red);
  nh.getParam("/FIRA/HSV/Blue", HSV_blue);
  nh.getParam("/FIRA/HSV/Yellow", HSV_yellow);
  nh.getParam("/FIRA/HSV/Green", HSV_green);
  for (int b = 0; b < 256; b++) {
    for (int g = 0; g < 256; g++) {
      for (int r = 0; r < 256; r++) {
        double R, G, B, H_sum, S_sum, V_sum;
        B = b / 255.0;
        G = g / 255.0;
        R = r / 255.0;
        double Max = (max(R, G) > max(G, B)) ? max(R, G) : max(G, B);
        double Min = (min(R, G) < min(G, B)) ? min(R, G) : min(G, B);
        if (Max == Min) {Max += 1;}
        if (R == Max) {H_sum = (G - B) * 60 / (Max - Min);}
        if (G == Max) {H_sum = 120 + (B - R) * 60 / (Max - Min);}
        if (B == Max) {H_sum = 240 + (R - G) * 60 / (Max - Min);}
        if (B == G && B == R) {H_sum = 0;}
        if (H_sum < 0) {H_sum = H_sum + 360;}
        if (Max == 0) {S_sum = 0;}
        S_sum = (((Max - Min) * 100) / Max);
        V_sum = Max * 100;
        HSVmap[r + (g << 8) + (b << 16)] = 0x00;
        if (HSV_red[0] < HSV_red[1]) {
          if ( (H_sum >= HSV_red[0]) && (H_sum <= HSV_red[1])
               && (S_sum >= HSV_red[2]) && (S_sum <= HSV_red[3])
               && (V_sum >= HSV_red[4]) && (V_sum <= HSV_red[5]) )
            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | REDITEM;

        } else {
          if ( (H_sum >= HSV_red[0]) || (H_sum <= HSV_red[1])
               && (S_sum >= HSV_red[2]) && (S_sum <= HSV_red[3])
               && (V_sum >= HSV_red[4]) && (V_sum <= HSV_red[5]) )
            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | REDITEM;
        }
        if (HSV_green[0] < HSV_green[1]) {
          if ( (H_sum >= HSV_green[0]) && (H_sum <= HSV_green[1])
               && (S_sum >= HSV_green[2]) && (S_sum <= HSV_green[3])
               && (V_sum >= HSV_green[4]) && (V_sum <= HSV_green[5]) )
            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | GREENITEM;
        } else {
          if ( (H_sum >= HSV_green[0]) || (H_sum <= HSV_green[1])
               && (S_sum >= HSV_green[2]) && (S_sum <= HSV_green[3])
               && (V_sum >= HSV_green[4]) && (V_sum <= HSV_green[5]) )
            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | GREENITEM;
        }
        if (HSV_blue[0] < HSV_blue[1]) {
          if ( (H_sum >= HSV_blue[0]) && (H_sum <= HSV_blue[1])
               && (S_sum >= HSV_blue[2]) && (S_sum <= HSV_blue[3])
               && (V_sum >= HSV_blue[4]) && (V_sum <= HSV_blue[5]) )
            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | BLUEITEM;
        } else {
          if ( (H_sum >= HSV_blue[0]) || (H_sum <= HSV_blue[1])
               && (S_sum >= HSV_blue[2]) && (S_sum <= HSV_blue[3])
               && (V_sum >= HSV_blue[4]) && (V_sum <= HSV_blue[5]) )
            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | BLUEITEM;
        }
        if (HSV_yellow[0] < HSV_yellow[1]) {
          if ( (H_sum >= HSV_yellow[0]) && (H_sum <= HSV_yellow[1])
               && (S_sum >= HSV_yellow[2]) && (S_sum <= HSV_yellow[3])
               && (V_sum >= HSV_yellow[4]) && (V_sum <= HSV_yellow[5]) )
            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | YELLOWITEM;
        } else {
          if ( (H_sum >= HSV_yellow[0]) || (H_sum <= HSV_yellow[1])
               && (S_sum >= HSV_yellow[2]) && (S_sum <= HSV_yellow[3])
               && (V_sum >= HSV_yellow[4]) && (V_sum <= HSV_yellow[5]) )
            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | YELLOWITEM;
        }
        if (HSV_white[0] < HSV_white[1]) {
          if ( (H_sum >= HSV_white[0]) && (H_sum <= HSV_white[1])
               && (S_sum >= HSV_white[2]) && (S_sum <= HSV_white[3])
               && (V_sum >= HSV_white[4]) && (V_sum <= HSV_white[5]) )
            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | WHITEITEM;
        } else {
          if ( (H_sum >= HSV_white[0]) || (H_sum <= HSV_white[1])
               && (S_sum >= HSV_white[2]) && (S_sum <= HSV_white[3])
               && (V_sum >= HSV_white[4]) && (V_sum <= HSV_white[5]) )
            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | WHITEITEM;
        }
      }
    }
  }
  string Filename = vision_path + FILE_PATH;
  const char *Filename_Path = Filename.c_str();  
  //cout << HSV_blue[0] << endl;
  if (SaveButton != 0) {
    FILE *file = fopen(Filename_Path, "rb+"); //開啟檔案來寫
    fwrite( HSVmap, 1, 256 * 256 * 256 , file );
    fclose(file);
    SaveButton = 0;
  }
}
