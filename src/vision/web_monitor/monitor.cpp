#define PI 3.14159265
#include "monitor.hpp"
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

std::string visionpath = ros::package::getPath("vision");

std::string defaultpath = "/config/default.yaml";
std::string parameterpath = "/config/Parameter.yaml";
std::string def = visionpath + defaultpath;
std::string param = visionpath + parameterpath; 
const char *defpath = def.c_str();
const char *parampath = param.c_str();


void InterfaceProc::Parameter_getting(const int x)
{
  if(ifstream(parampath)){
    cout<<visionpath<<endl;
    std::string temp = "rosparam load " + param; 
    const char *load = temp.c_str(); 
    system(load);
    cout<<"Read the yaml file"<<endl;
    nh.getParam("/FIRA/HSV/Ball",HSV_red);
    nh.getParam("/FIRA/HSV/Blue",HSV_blue);
    nh.getParam("/FIRA/HSV/Yellow",HSV_yellow);
    nh.getParam("/FIRA/HSV/Green",HSV_green);
    nh.getParam("/FIRA/HSV/White",HSV_white);
    nh.getParam("/FIRA/HSV/ColorMode",ColorModeMsg);
    nh.getParam("/FIRA/HSV/white/gray",WhiteGrayMsg);
    nh.getParam("/FIRA/HSV/white/angle",WhiteAngleMsg);
    nh.getParam("/FIRA/HSV/black/gray",BlackGrayMsg);
    nh.getParam("/FIRA/HSV/black/angle",BlackAngleMsg);

  /////////////////////////////////掃瞄點前置參數///////////////////////////////////
    nh.getParam("/FIRA/SCAN/Angle_Near_Gap",Angle_Near_GapMsg);
    nh.getParam("/FIRA/SCAN/Magn_Near_Gap",Magn_Near_GapMsg);
    nh.getParam("/FIRA/SCAN/Magn_Near_Start",Magn_Near_StartMsg);
    nh.getParam("/FIRA/SCAN/Magn_Middle_Start",Magn_Middle_StartMsg);
    nh.getParam("/FIRA/SCAN/Magn_Far_Start",Magn_Far_StartMsg);
    nh.getParam("/FIRA/SCAN/Magn_Far_End",Magn_Far_EndMsg);
    nh.getParam("/FIRA/SCAN/Dont_Search_Angle_1",Dont_Search_Angle_1Msg);
    nh.getParam("/FIRA/SCAN/Dont_Search_Angle_2",Dont_Search_Angle_2Msg);
    nh.getParam("/FIRA/SCAN/Dont_Search_Angle_3",Dont_Search_Angle_3Msg);
    nh.getParam("/FIRA/SCAN/Angle_range_1",Angle_range_1Msg);
    nh.getParam("/FIRA/SCAN/Angle_range_2_3",Angle_range_2_3Msg);

    search_angle    = Angle_Near_GapMsg;
    search_distance = Magn_Near_GapMsg;
    search_start    = Magn_Near_StartMsg;
    search_near     = Magn_Middle_StartMsg;
    search_middle   = Magn_Far_StartMsg;
    search_end      = Magn_Far_EndMsg;

    dont_angle[0] = Dont_Search_Angle_1Msg;
    dont_angle[1] = Dont_Search_Angle_2Msg;
    dont_angle[2] = Dont_Search_Angle_3Msg;
    dont_angle[3] = Angle_range_1Msg;
    dont_angle[4] = Angle_range_2_3Msg;
    dont_angle[5] = Angle_range_2_3Msg;
  ///////////////////////////////////////FPS設定////////////////////////////////////////////////
    nh.getParam("/FIRA/FPS",fpsMsg);
    get_campara();
  //////////////////////////////////// CNETER設定///////////////////////////////////////////////
    nh.getParam("/FIRA/Center/Center_X",CenterXMsg);
    nh.getParam("/FIRA/Center/Center_Y",CenterYMsg);
    nh.getParam("/FIRA/Center/Inner",InnerMsg);
    nh.getParam("/FIRA/Center/Outer",OuterMsg);
    nh.getParam("/FIRA/Center/Front",FrontMsg);
    nh.getParam("/FIRA/Center/Camera_high",Camera_HighMsg);

    center_x=CenterXMsg;
    center_y=CenterYMsg;
    center_inner=InnerMsg;
    center_outer=OuterMsg;
    center_front=FrontMsg;
    Camera_H=Camera_HighMsg;

    nh.getParam("/FIRA/Parameterbutton",buttonmsg);
    cout<<center_x<<endl;
  }
}
void InterfaceProc::SaveButton_setting(const vision::bin msg)
{
  
  SaveButton = msg.bin;
  Parameter_getting(1);
  HSVmap();
}


InterfaceProc::InterfaceProc()
    :it_(nh) 
{
  ros::NodeHandle n("~");	
  Parameter_getting(1);	
  init_data();
  //image_sub_ = it_.subscribe("/camera/image_raw", 1, &InterfaceProc::imageCb, this);
  image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &InterfaceProc::imageCb, this);
  image_pub_threshold_ = it_.advertise("/camera/image_monitor", 1);//http://localhost:8080/stream?topic=/camera/image_monitor webfor /camera/image
  s1 = nh.subscribe("interface/bin_save",1000, &InterfaceProc::SaveButton_setting,this);
  object_pub = nh.advertise<vision::Object>("/vision/object",1);
  Two_point_pub = nh.advertise<vision::Two_point>("/interface/Two_point",1);
} 
InterfaceProc::~InterfaceProc()
{
  delete frame;
  cv::destroyWindow(OPENCV_WINDOW);
}
/////////////////////////////////影像讀進來//////////////////////////////////////////
void InterfaceProc::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  }catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
//////////////////////Clone///////////////////////////////////
  cv::flip(cv_ptr->image, cv_ptr->image, 1);
  Main_frame=cv_ptr->image.clone();
//////////////////////////////////////////////////////////////
  vision_path = ros::package::getPath("vision");
  color_map = ColorFile();
  double ang_PI;
  for(int ang=0 ; ang<360; ang++){
    ang_PI = ang*PI/180;
    Angle_sin.push_back(sin(ang_PI));
    Angle_cos.push_back(cos(ang_PI));
  }

  cv::waitKey(3);
  Obstaclemap = Mat(Size(Main_frame.cols,Main_frame.rows),CV_8UC3,Scalar(0,0,0));

  object_Item_reset(Red_Item);
  object_Item_reset(Blue_Item);
  object_Item_reset(Yellow_Item);

  objectdet_change(Findmap,REDITEM,Red_Item);
  objectdet_change(Findmap,BLUEITEM,Blue_Item);
  objectdet_change(Findmap,YELLOWITEM,Yellow_Item);

  Obstacle_Item = new object_Item [5];

  object_Item_reset(Obstacle_Item[0]);
  object_Item_reset(Obstacle_Item[1]);
  object_Item_reset(Obstacle_Item[2]);
  object_Item_reset(Obstacle_Item[3]);
  object_Item_reset(Obstacle_Item[4]);
  creat_Obstclemap(Obstaclemap,OBSTACLEITEM);
  creat_FIRA_map(Obstaclemap,FIRA_map);	
  objectdet_Obstacle(Findmap,OBSTACLEITEM,Obstacle_Item);

  vision::Object object_msg;

  if(Red_Item.distance!=0){
    object_msg.ball_x = Red_Item.x-CenterXMsg;
    object_msg.ball_y = 0-(Red_Item.y-CenterYMsg);
    object_msg.ball_LR = Red_Item.LR;
    object_msg.ball_ang = Red_Item.angle;
    object_msg.ball_dis = Omni_distance(Red_Item.distance);
  }else{
    object_msg.ball_ang = 999;
    object_msg.ball_dis = 999;
  }

  if(Blue_Item.distance!=0){
    object_msg.blue_x = Blue_Item.x-CenterXMsg;
    object_msg.blue_y = 0-(Blue_Item.y-CenterYMsg);
    object_msg.blue_LR = Blue_Item.LR;
    object_msg.blue_ang = Blue_Item.angle;
    object_msg.blue_dis = Omni_distance(Blue_Item.distance);
  }else{
    object_msg.blue_ang = 999;
    object_msg.blue_dis = 999;
  }

  if(Yellow_Item.distance!=0){
    object_msg.yellow_x = Yellow_Item.x-CenterXMsg;
    object_msg.yellow_y = 0-(Yellow_Item.y-CenterYMsg);
    object_msg.yellow_LR = Yellow_Item.LR;
    object_msg.yellow_ang = Yellow_Item.angle;
    object_msg.yellow_dis = Omni_distance(Yellow_Item.distance);
  }else{
    object_msg.yellow_ang = 999;
    object_msg.yellow_dis = 999;
  }
/////////////////////FPS///////////////////////
  frame_counter++;
  static long int StartTime = time(NULL);//ros::Time::now().toNSec();
  static long int EndTime;
  static long double FrameRate = 0.0;

//time(NULL);
  if(frame_counter == 17){
    EndTime = time(NULL);//ros::Time::now().toNSec();
    dt = (EndTime - StartTime)*10000/frame_counter;
    StartTime = EndTime;
    EndTime = 0;
    if( dt!=0 )
    {
      //FrameRate = ( 1000000000.0 / dt ) * ALPHA + FrameRate * ( 1.0 - ALPHA );
      FrameRate = ( 10000.0 / dt ) + FrameRate * ( 1.0 - ALPHA );
      //cout << "FPS: " << FrameRate << endl;
    }
    frame_counter = 0;
    //dt = 0;
  }
  object_msg.fps = FrameRate;
///////////////////////////////////////////////
  Findmap.release();
  FIRA_map.release();
  Obstaclemap.release();
  Erodemap.release();
  Dilatemap.release();
   //object_pub.publish(object_msg);
  topic_counter++;
  if(topic_counter==10){
  object_pub.publish(object_msg);
  topic_counter=0;
}
  //imshow(OPENCV_WINDOW, Main_frame);

 if(buttonmsg == 7){
 sensor_msgs::ImagePtr thresholdMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Main_frame).toImageMsg();
  image_pub_threshold_.publish(thresholdMsg);
}
   cv::waitKey(3);




}
//////////////////////處理影像開始//////////////////////////////////////
 
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
double InterfaceProc::camera_f(double Omni_pixel)
{
  double m = (Omni_pixel*0.0099)/60;        // m = H1/H0 = D1/D0    D0 + D1 = 180
  double D0 = 180/(1+m);                    // D1 = m   *D0
  double D1 = 180/(1+(1/m));                // D0 = 1/m *D1
  double f = 1/(1/D0 + 1/D1);
  //ROS_INFO("m = %f D0 = %f D1 = %f F = %f",m,D0,D1,f);
  return D1;
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
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////掃描參數(Scan)//////////////////////////////////
//掃描點座標調整
//修正超出圖片的點座標
int Frame_Area(int coordinate, int range)
{
  if(coordinate < 0) coordinate = 0;
  else if(coordinate >= range) coordinate = range -1;
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
  if(radius <= Magn_Middle_StartMsg) return Angle_Near_GapMsg;
  else if(radius > Magn_Middle_StartMsg && radius <= Magn_Far_StartMsg) return Angle_Near_GapMsg /2;
  else return Angle_Near_GapMsg /4;
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

      Two_point_msg.blue_dis = Blue_Item.dis_min;
      Two_point_msg.blue_ang1 = blue_angle_max;
      Two_point_msg.blue_ang2 = blue_angle_min;}

  if(color = YELLOWITEM){
      yellow_angle_max = Angle_Adjustment(Yellow_Item.ang_max);
      yellow_angle_min = Angle_Adjustment(Yellow_Item.ang_min);

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
////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////HSVmap////////////////////////////////
void InterfaceProc::HSVmap()
{
     unsigned char *HSVmap = new unsigned char[256*256*256];
     nh.getParam("/FIRA/HSV/Ball",HSV_red);
      nh.getParam("/FIRA/HSV/Blue",HSV_blue);
      nh.getParam("/FIRA/HSV/Yellow",HSV_yellow);
      nh.getParam("/FIRA/HSV/Green",HSV_green);
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
                if(HSV_red[0] < HSV_red[1]){
                    if( (H_sum >= HSV_red[0]) && (H_sum <= HSV_red[1])
                      &&(S_sum >= HSV_red[2]) && (S_sum <= HSV_red[3])
                      &&(V_sum >= HSV_red[4]) && (V_sum <= HSV_red[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | REDITEM;

                }else{
                    if( (H_sum >= HSV_red[0]) || (H_sum <= HSV_red[1])
                      &&(S_sum >= HSV_red[2]) && (S_sum <= HSV_red[3])
                      &&(V_sum >= HSV_red[4]) && (V_sum <= HSV_red[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | REDITEM;
                }
                if(HSV_green[0] < HSV_green[1]){
                    if( (H_sum >= HSV_green[0]) && (H_sum <= HSV_green[1])
                      &&(S_sum >= HSV_green[2]) && (S_sum <= HSV_green[3])
                      &&(V_sum >= HSV_green[4]) && (V_sum <= HSV_green[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | GREENITEM;
                }else{
                    if( (H_sum >= HSV_green[0]) || (H_sum <= HSV_green[1])
                      &&(S_sum >= HSV_green[2]) && (S_sum <= HSV_green[3])
                      &&(V_sum >= HSV_green[4]) && (V_sum <= HSV_green[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | GREENITEM;
                }
                if(HSV_blue[0] < HSV_blue[1]){
                    if( (H_sum >= HSV_blue[0]) && (H_sum <= HSV_blue[1])
                      &&(S_sum >= HSV_blue[2]) && (S_sum <= HSV_blue[3])
                      &&(V_sum >= HSV_blue[4]) && (V_sum <= HSV_blue[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | BLUEITEM;
                }else{
                    if( (H_sum >= HSV_blue[0]) || (H_sum <= HSV_blue[1])
                      &&(S_sum >= HSV_blue[2]) && (S_sum <= HSV_blue[3])
                      &&(V_sum >= HSV_blue[4]) && (V_sum <= HSV_blue[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | BLUEITEM;
                }
                if(HSV_yellow[0] < HSV_yellow[1]){
                    if( (H_sum >= HSV_yellow[0]) && (H_sum <= HSV_yellow[1])
                      &&(S_sum >= HSV_yellow[2]) && (S_sum <= HSV_yellow[3])
                      &&(V_sum >= HSV_yellow[4]) && (V_sum <= HSV_yellow[5]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | YELLOWITEM;
                }else{
                    if( (H_sum >= HSV_yellow[0]) || (H_sum <= HSV_yellow[1])
                      &&(S_sum >= HSV_yellow[2]) && (S_sum <= HSV_yellow[3])
                      &&(V_sum >= HSV_yellow[4]) && (V_sum <= HSV_yellow[5]) )
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












