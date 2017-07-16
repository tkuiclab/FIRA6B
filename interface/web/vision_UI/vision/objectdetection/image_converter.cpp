#include "image_converter.hpp"
#include "math.h"

#define PI 3.14159265
#define REDITEM 0x01
#define GREENITEM 0x02
#define BLUEITEM 0x04
#define YELLOWITEM 0x08
#define WHITEITEM 0x10
#define OBSTACLEITEM 0x00
#define FILE_PATH "/config/HSVcolormap.bin"

using namespace std;
using namespace cv;
typedef unsigned char BYTE;
const double ALPHA = 0.5;

ImageConverter::ImageConverter()
   :it_(nh)
{
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
    object_pub = nh.advertise<vision::Object>("/vision/object",1);

    vision_path = ros::package::getPath("vision");

    color_map = ColorFile();

    get_Camera();
    get_center();
    get_scan();
    frame_counter = 0;

    double ang_PI;

    for(int ang=0 ; ang<360; ang++){
      ang_PI = ang*M_PI/180;
      Angle_sin.push_back(sin(ang_PI));
      Angle_cos.push_back(cos(ang_PI));
    }

    search_angle    = scan_para[0];
    search_distance = scan_para[1];
    search_start    = scan_para[2];
    search_near     = scan_para[3];
    search_middle   = scan_para[4];
    search_end      = scan_para[5];

    dont_angle[0] = scan_para[6];
    dont_angle[1] = scan_para[7];
    dont_angle[2] = scan_para[8];
    dont_angle[3] = scan_para[9];
    dont_angle[4] = scan_para[10];
    dont_angle[5] = scan_para[11];
}

ImageConverter::~ImageConverter()
{

}

//////////////////////////////////////////////////參數//////////////////////////////////////////////////

void ImageConverter::get_center(){
    nh.getParam("/FIRA/Center/X",center_x);
    nh.getParam("/FIRA/Center/Y",center_y);
    nh.getParam("/FIRA/Center/Inner",center_inner);
    nh.getParam("/FIRA/Center/Outer",center_outer);
    nh.getParam("/FIRA/Center/Front",center_front);
}
void ImageConverter::get_scan(){
    nh.getParam("/FIRA/Scan/Parameter",scan_para);
}
void ImageConverter::get_Camera(){
    nh.getParam("/FIRA/Camera/High",Camera_H);
    nh.getParam("/FIRA/Camera/Focal",Camera_f);
}

//////////////////////////////////////////////////影像進來//////////////////////////////////////////////////

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
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

    cv::flip(cv_ptr->image, Main_frame, 1);

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
      object_msg.ball_x = Red_Item.x;
      object_msg.ball_y = Red_Item.y;
      object_msg.ball_LR = Red_Item.LR;
      object_msg.ball_ang = Red_Item.angle;
      object_msg.ball_dis = Omni_distance(Red_Item.distance);
    }else{
      object_msg.ball_ang = 999;
      object_msg.ball_dis = 999;
    }

    if(Blue_Item.distance!=0){
      object_msg.blue_x = Blue_Item.x;
      object_msg.blue_y = Blue_Item.y;
      object_msg.blue_LR = Blue_Item.LR;
      object_msg.blue_ang = Blue_Item.angle;
      object_msg.blue_dis = Omni_distance(Blue_Item.distance);
    }else{
      object_msg.blue_ang = 999;
      object_msg.blue_dis = 999;
    }

    if(Yellow_Item.distance!=0){
      object_msg.yellow_x = Yellow_Item.x;
      object_msg.yellow_y = Yellow_Item.y;
      object_msg.yellow_LR = Yellow_Item.LR;
      object_msg.yellow_ang = Yellow_Item.angle;
      object_msg.yellow_dis = Omni_distance(Yellow_Item.distance);
    }else{
      object_msg.yellow_ang = 999;
      object_msg.yellow_dis = 999;
    }

//    Mat element = getStructuringElement(MORPH_RECT,Size(2,2));

//    cv::dilate(Obstaclemap, Dilatemap, element);
//    cv::erode(Obstaclemap, Erodemap, element);

//    cv::imshow("Dilate",Dilatemap);
//    cv::waitKey(1);
//    cv::imshow("Erodemap",Erodemap);
//    cv::waitKey(1);
//    cv::imshow("Image", Main_frame);
//    cv::waitKey(1);

    ///////////////////////////////////////////////
    /////////////////////FPS///////////////////////

    frame_counter++;
    static long int StartTime = ros::Time::now().toNSec();
    static double FrameRate = 0.0;

    if(frame_counter == 10){
      EndTime = ros::Time::now().toNSec();
      dt = (EndTime - StartTime)/frame_counter;
      StartTime = EndTime;
      if( dt!=0 )
      {
              FrameRate = ( 1000000000.0 / dt ) * ALPHA + FrameRate * ( 1.0 - ALPHA );
              //cout << "FPS: " << FrameRate << endl;
      }
      frame_counter = 0;
    }
    object_msg.fps = FrameRate;

    ///////////////////////////////////////////////

    Findmap.release();
    FIRA_map.release();
    Obstaclemap.release();
    Erodemap.release();
    Dilatemap.release();

    object_pub.publish(object_msg);
//    ros::spinOnce();
}

//////////////////////////////////////////////////色彩空間//////////////////////////////////////////////////
vector<BYTE> ImageConverter::ColorFile()
{
  string Filename = vision_path+FILE_PATH;
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

//////////////////////////////////////////////////物件分割//////////////////////////////////////////////////
int Frame_area(int num,int range){
  if(num < 0) num = 0;
  else if(num >= range) num = range-1;
  return num;
}

int Planning_angle(int ang,int angle){
  if (ang < 0) return ang+angle;
  else if (ang >= angle) return ang-angle;
  else return ang;
}

int ImageConverter::angle_for_distance(int dis){
  if(dis <= search_near) return search_angle;
  else if(dis > search_near && dis <= search_middle) return search_angle/2;
  else return search_angle/4;
}

void ImageConverter::objectdet_change(Mat &frame_, int color, object_Item &obj_item){
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
        angle += angle_for_distance(distance);
        continue;
      }

      object_size = 0;
      FIND_Item.size = 0;

      x_= distance*Angle_cos[angle];
      y_= distance*Angle_sin[angle];

      x = Frame_area(center_x+x_,frame_.cols);
      y = Frame_area(center_y-y_,frame_.rows);

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

      angle += angle_for_distance(distance);
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

  draw_ellipse(Main_frame,Red_Item);
  draw_ellipse(Main_frame,Yellow_Item);
  draw_ellipse(Main_frame,Blue_Item);
}

void ImageConverter::creat_Obstclemap(Mat &frame_, int color){
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
        angle += angle_for_distance(distance);
        continue;
      }

      x_= distance*Angle_cos[angle];
      y_= distance*Angle_sin[angle];

      x = Frame_area(center_x+x_,frame_.cols);
      y = Frame_area(center_y-y_,frame_.rows);

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

/*  if(color == OBSTACLEITEM){
    cv::imshow("obstacle_map", frame_);
    cv::waitKey(1);
  }
*/
}

void ImageConverter::creat_FIRA_map(Mat &frame_input , Mat &frame_output){
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

      x = Frame_area(center_x+x_,frame_input.cols);
      y = Frame_area(center_y-y_,frame_input.rows);

      if(frame_input.data[(y*frame_input.cols+x)*3+0]==255){
        distance_get = 1;

        dis = Omni_distance(distance);

        angle_ = Planning_angle(angle-center_front+90,360);

        x_= dis*Angle_cos[angle_];
        y_= dis*Angle_sin[angle_];

        x = Frame_area(frame_center_x+x_,frame_output.cols);
        y = Frame_area(frame_center_y-y_,frame_output.rows);

        points[angle] = Point(x, y);

//        frame_output.data[y*frame_output.cols+x] = 255;

        break;
      }
    }
    if(distance_get == 0){
      dis = Omni_distance(search_end);

      angle_ = Planning_angle(angle-center_front+90,360);

      x_= dis*Angle_cos[angle_];
      y_= dis*Angle_sin[angle_];

      x = Frame_area(frame_center_x+x_,frame_output.cols);
      y = Frame_area(frame_center_y-y_,frame_output.rows);

      points[angle] = Point(x, y);

//      frame_output.data[y*frame_output.cols+x] = 255;
    }
    distance_get = 0;
  }

  for(int i=0 ;i<360;i++){
    line(frame_output, points[i], points[Planning_angle(i+1,360)], 255, 1);
  }

  /*cv::imshow("FIRA_map", FIRA_map);
  cv::waitKey(1);
*/
}

void ImageConverter::objectdet_Obstacle(Mat &frame_, int color, object_Item *obj_item){
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

      x = Frame_area(center_x+x_,frame_.cols);
      y = Frame_area(center_y-y_,frame_.rows);

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

      angle += angle_for_distance(distance);
    }
  }

  find_object_point(*(obj_item+0),color);
  find_object_point(*(obj_item+1),color);
  find_object_point(*(obj_item+2),color);
  find_object_point(*(obj_item+3),color);
  find_object_point(*(obj_item+4),color);

//  if(color == OBSTACLEITEM){
//    draw_ellipse(frame_,Obstacle_Item[0]);
//    draw_ellipse(frame_,Obstacle_Item[1]);
//    draw_ellipse(frame_,Obstacle_Item[2]);
//    draw_ellipse(frame_,Obstacle_Item[3]);
//    draw_ellipse(frame_,Obstacle_Item[4]);
//    cv::imshow("obstacle", frame_);
//    cv::waitKey(1);
//  }
}

void ImageConverter::Mark_point(Mat &frame_, int distance, int angle, int x, int y, int &size, int color){
    frame_.data[(y*frame_.cols+x)*3+0] = 255;
    frame_.data[(y*frame_.cols+x)*3+1] = 255;
    frame_.data[(y*frame_.cols+x)*3+2] = 255;
    find_point.push_back(distance);
    find_point.push_back(angle);
    size += 1;
}

void ImageConverter::object_Item_reset(object_Item &obj_){
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

void ImageConverter::find_around(Mat &frame_, int distance ,int angle, int &size, int color){
  int x,y;
  int x_,y_;
  int dis_f,ang_f;
  double angle_f;

  for(int i=-1 ; i<2 ; i++){
    for(int j=-1 ; j<2 ; j++){
      dis_f = distance + i*search_distance;

      if(dis_f < search_start) dis_f = search_start;

      if(color == REDITEM || color == BLUEITEM || color == YELLOWITEM){
        dis_f = Frame_area(dis_f,search_end);
      }else if(color == OBSTACLEITEM){
        dis_f = Frame_area(dis_f,search_middle);
      }

      ang_f = angle + j*angle_for_distance(dis_f);

      while(Planning_angle(ang_f,360) > dont_angle[0] && Planning_angle(ang_f,360) < dont_angle[1] ||
         Planning_angle(ang_f,360) > dont_angle[2] && Planning_angle(ang_f,360) < dont_angle[3] ||
         Planning_angle(ang_f,360) > dont_angle[4] && Planning_angle(ang_f,360) < dont_angle[5]) {
        ang_f += j*angle_for_distance(dis_f);
      }

      angle_f = Planning_angle(ang_f,360);

      x_= dis_f*Angle_cos[angle_f];
      y_= dis_f*Angle_sin[angle_f];

      x = Frame_area(center_x+x_,frame_.cols);
      y = Frame_area(center_y-y_,frame_.rows);

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

void ImageConverter::object_compare(int distance ,int angle){
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

void ImageConverter::draw_ellipse(Mat &frame_, object_Item &obj_){
  ellipse(frame_, Point(center_x,center_y), Size(obj_.dis_min,obj_.dis_min), 0, 360-obj_.ang_max, 360-obj_.ang_min, Scalar(255,255,0), 1);
  ellipse(frame_, Point(center_x,center_y), Size(obj_.dis_max,obj_.dis_max), 0, 360-obj_.ang_max, 360-obj_.ang_min, Scalar(255,255,0), 1);
  draw_Line(frame_, obj_.dis_max, obj_.dis_min, obj_.ang_max);
  draw_Line(frame_, obj_.dis_max, obj_.dis_min, obj_.ang_min);

  circle(frame_, Point(obj_.x,obj_.y), 2, Scalar(0,0,255), -1);
}

void ImageConverter::draw_Line(Mat &frame_, int obj_distance_max, int obj_distance_min, int obj_angle){
  int x_,y_;
  double angle_f;
  int x[2],y[2];

  angle_f = Planning_angle(obj_angle,360);

  x_= obj_distance_min*Angle_cos[angle_f];
  y_= obj_distance_min*Angle_sin[angle_f];

  x[0] = Frame_area(center_x+x_,frame_.cols);
  y[0] = Frame_area(center_y-y_,frame_.rows);

  x_= obj_distance_max*Angle_cos[angle_f];
  y_= obj_distance_max*Angle_sin[angle_f];

  x[1] = Frame_area(center_x+x_,frame_.cols);
  y[1] = Frame_area(center_y-y_,frame_.rows);

  line(frame_, Point(x[0],y[0]), Point(x[1],y[1]), Scalar(255,255,0), 1);
}
//////////////////////////////////////////////////中心點計算//////////////////////////////////////////////////
void ImageConverter::find_object_point(object_Item &obj_, int color){
  int x,y;
  int x_,y_;

  int angle_,distance_;
  int angle_range;
  int find_angle;

  unsigned char B,G,R;

  if(color == REDITEM){
    angle_= Planning_angle((obj_.ang_max + obj_.ang_min)/2,360);
    distance_ = (obj_.dis_max + obj_.dis_min) / 2;

    find_angle = Planning_angle(angle_,360);

    x_= distance_*Angle_cos[find_angle];
    y_= distance_*Angle_sin[find_angle];

    x = Frame_area(center_x+x_,Main_frame.cols);
    y = Frame_area(center_y-y_,Main_frame.rows);

    obj_.x = x;
    obj_.y = y;
    obj_.distance = sqrt(pow(x_,2)+pow(y_,2));

  }else if(color == BLUEITEM || color == YELLOWITEM){
    angle_= Planning_angle((obj_.ang_max + obj_.ang_min)/2,360);
    angle_range = 0.7*Planning_angle((obj_.ang_max - obj_.ang_min)/2,360);

    for(int distance = obj_.dis_min ; distance <= obj_.dis_max ; distance++){
      for(int angle = 0 ; angle <= angle_range ; angle++){
        find_angle = Planning_angle(angle_ + angle,360);

        x_= distance*Angle_cos[find_angle];
        y_= distance*Angle_sin[find_angle];

        x = Frame_area(center_x+x_,Main_frame.cols);
        y = Frame_area(center_y-y_,Main_frame.rows);

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

        find_angle = Planning_angle(angle_ - angle,360);

        x_= distance*Angle_cos[find_angle];
        y_= distance*Angle_sin[find_angle];

        x = Frame_area(center_x+x_,Main_frame.cols);
        y = Frame_area(center_y-y_,Main_frame.rows);

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
    angle_= Planning_angle((obj_.ang_max + obj_.ang_min)/2,360);
    angle_range = Planning_angle((obj_.ang_max - obj_.ang_min)/2,360);

    for(int distance = obj_.dis_min ; distance <= obj_.dis_max ; distance++){
      for(int angle = 0 ; angle <= angle_range ; angle++){
        find_angle = Planning_angle(angle_ + angle,360);

        x_= distance*Angle_cos[find_angle];
        y_= distance*Angle_sin[find_angle];

        x = Frame_area(center_x+x_,Main_frame.cols);
        y = Frame_area(center_y-y_,Main_frame.rows);

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

        find_angle = Planning_angle(angle_ - angle,360);

        x_= distance*Angle_cos[find_angle];
        y_= distance*Angle_sin[find_angle];

        x = Frame_area(center_x+x_,Main_frame.cols);
        y = Frame_area(center_y-y_,Main_frame.rows);

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

  if(Planning_angle(angle_-center_front,360) < 180){
    obj_.LR = "Left";
    obj_.angle = Planning_angle(angle_-center_front,360);
  }else{
    obj_.LR = "Right";
    obj_.angle = Planning_angle(angle_-center_front,360)-360;
  }
}

//////////////////////////////////////////////////距離計算//////////////////////////////////////////////////

double ImageConverter::Omni_distance(double dis_pixel){
  double Z = -1*Camera_H;
  double c = 83.125;
  double b = c*0.8722;

  double f = Camera_f;

  double dis;

  //double pixel_dis = sqrt(pow(object_x,2)+pow(object_y,2));

  double pixel_dis = dis_pixel;

  double r = atan2(f,pixel_dis*0.0099);

  dis = Z*(pow(b,2)-pow(c,2))*cos(r) / ((pow(b,2)+pow(c,2))*sin(r) - 2*b*c);

  if(dis/10 < 0 || dis/10 > 999){
    dis = 9990;
  }

  return dis/10;
}


