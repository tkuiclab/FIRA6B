#include "image_converter.hpp"
#include "math.h"

using namespace std;
using namespace cv;
const double ALPHA = 0.5;

ImageConverter::ImageConverter()
   :it_(nh)
{
  get_Camera();
  get_center();
  get_distance();
  get_whitedata();
  image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
  black_pub  = nh.advertise<std_msgs::Int32MultiArray>("/vision/BlackRealDis",1);
  frame_counter = 0;

  double ang_PI;

  for(int ang=0 ; ang<360; ang++){
    ang_PI = ang*M_PI/180;
    Angle_sin.push_back(sin(ang_PI));
    Angle_cos.push_back(cos(ang_PI));
  }
} 

ImageConverter::~ImageConverter()
{

}

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

  for(int i=0;i<Main_frame.rows;i++){
      for(int j=0;j<Main_frame.cols;j++){
          unsigned char gray = ( Main_frame.data[(i*Main_frame.cols*3)+(j*3)+0]
                               + Main_frame.data[(i*Main_frame.cols*3)+(j*3)+1]
                               + Main_frame.data[(i*Main_frame.cols*3)+(j*3)+2])/3;
          if(gray < black_gray){
              Main_frame.data[(i*Main_frame.cols*3)+(j*3)+0] = 0;
              Main_frame.data[(i*Main_frame.cols*3)+(j*3)+1] = 0;
              Main_frame.data[(i*Main_frame.cols*3)+(j*3)+2] = 0;
          }else{
              Main_frame.data[(i*Main_frame.cols*3)+(j*3)+0] = 255;
              Main_frame.data[(i*Main_frame.cols*3)+(j*3)+1] = 255;
              Main_frame.data[(i*Main_frame.cols*3)+(j*3)+2] = 255;
          }
      }
  }

  blackItem_pixel.clear();
  BlackRealDis.data.clear();

  for(int angle = 0; angle < 360; angle = angle + black_angle){
      int angle_be = angle+center_front;

      if(angle_be >= 360) angle_be -= 360;

      double x_ = Angle_cos[angle_be];
      double y_ = Angle_sin[angle_be];
      for(int r = center_inner; r <= center_outer; r++){
          int dis_x = x_*r;
          int dis_y = y_*r;
          if( Main_frame.data[((center_y-dis_y)*Main_frame.cols + center_x+dis_x)*3+0] == 0
            &&Main_frame.data[((center_y-dis_y)*Main_frame.cols + center_x+dis_x)*3+1] == 0
            &&Main_frame.data[((center_y-dis_y)*Main_frame.cols + center_x+dis_x)*3+2] == 0){
              blackItem_pixel.push_back(hypot(dis_x,dis_y));
              break;
          }else{
              Main_frame.data[((center_y-dis_y)*Main_frame.cols + center_x+dis_x)*3+0] = 0;
              Main_frame.data[((center_y-dis_y)*Main_frame.cols + center_x+dis_x)*3+1] = 0;
              Main_frame.data[((center_y-dis_y)*Main_frame.cols + center_x+dis_x)*3+2] = 255;
          }
      }
  }

  int object_dis;
  for(int j=0;j<blackItem_pixel.size();j++){
    object_dis = Omni_distance(blackItem_pixel[j]);
    BlackRealDis.data.push_back(object_dis);
  }
  black_pub.publish(BlackRealDis);

  /////////////////////Show view/////////////////
//  cv::imshow("Image", Main_frame);
//  cv::waitKey(1);
  ///////////////////////////////////////////////
  /////////////////////FPS///////////////////////

  static long int StartTime = ros::Time::now().toNSec();
  frame_counter++;

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
  ///////////////////////////////////////////////
}

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

void ImageConverter::get_center(){
    nh.getParam("/FIRA/Center/X",center_x);
    nh.getParam("/FIRA/Center/Y",center_y);
    nh.getParam("/FIRA/Center/Inner",center_inner);
    nh.getParam("/FIRA/Center/Outer",center_outer);
    nh.getParam("/FIRA/Center/Front",center_front);
}
void ImageConverter::get_distance(){
    nh.getParam("/FIRA/Distance/Gap",dis_gap);
    nh.getParam("/FIRA/Distance/Space",dis_space);
    nh.getParam("/FIRA/Distance/Pixel",dis_pixel);
}
void ImageConverter::get_Camera(){
    nh.getParam("/FIRA/Camera/High",Camera_H);
    nh.getParam("/FIRA/Camera/Focal",Camera_f);
}
void ImageConverter::get_whitedata(){
    nh.getParam("/FIRA/blackItem/gray",black_gray);
    nh.getParam("/FIRA/blackItem/angle",black_angle);
}









