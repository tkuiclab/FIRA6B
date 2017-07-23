#include "image_converter.hpp"
#include <ros/ros.h>
#include "math.h"

using namespace std;
using namespace cv;

ImageConverter::ImageConverter()
   :it_(nh)
{
    get_center();
    get_distance();
    get_whitedata();
    get_Camera();
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
    white_pub  = nh.advertise<std_msgs::Int32MultiArray>("/vision/whiteRealDis",1);
} 

ImageConverter::~ImageConverter(){}

int Frame_area(int num,int range){
  if(num < 0) num = 0;
  else if(num >= range) num = range-1;
  return num;
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    int StartTime = ros::Time::now().toNSec();
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


    ////////////////////////////////////////////////bbbbbbbbbbbbbbbbbb
    Mat frame;
    cv::flip(cv_ptr->image, frame, 1);

    for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
            unsigned char gray = ( frame.data[(i*frame.cols*3)+(j*3)+0]
                                 + frame.data[(i*frame.cols*3)+(j*3)+1]
                                 + frame.data[(i*frame.cols*3)+(j*3)+2])/3;
            if(gray< white_gray ){
                frame.data[(i*frame.cols*3)+(j*3)+0] = 0;
                frame.data[(i*frame.cols*3)+(j*3)+1] = 0;
                frame.data[(i*frame.cols*3)+(j*3)+2] = 0;
            }else{
                frame.data[(i*frame.cols*3)+(j*3)+0] = 255;
                frame.data[(i*frame.cols*3)+(j*3)+1] = 255;
                frame.data[(i*frame.cols*3)+(j*3)+2] = 255;
            }
        }
    }

    whiteline_pixel.clear();
    WhiteRealDis.data.clear();
    int timesqwer = 0;
    for(int a = 0; a < 360; a = a + white_angle){
        int angle_be = a+center_front;
        cout<<a<<endl;
        if(angle_be>360) angle_be -= 360;

        double angle_af = angle_be*M_PI/180;

        double x = cos(angle_af);
        double y = sin(angle_af);

        for(int r=center_inner;r<=center_outer;r++){
            int dis_x = x*r;
            int dis_y = y*r;

            int image_x = Frame_area(center_x+dis_x,frame.cols);
            int image_y = Frame_area(center_y-dis_y,frame.rows);

            if( frame.data[(image_y*frame.cols*3)+(image_x*3)+0] == 255
              &&frame.data[(image_y*frame.cols*3)+(image_x*3)+1] == 255
              &&frame.data[(image_y*frame.cols*3)+(image_x*3)+2] == 255){
                whiteline_pixel.push_back(hypot(dis_x,dis_y));
                break;
            }else{
//                frame.data[((center_y-dis_y)*frame.cols*3)+((center_x+dis_x)*3)+0] = 0;
//                frame.data[((center_y-dis_y)*frame.cols*3)+((center_x+dis_x)*3)+1] = 0;
//                frame.data[((center_y-dis_y)*frame.cols*3)+((center_x+dis_x)*3)+2] = 255;
                if(r==center_outer){
                    whiteline_pixel.push_back(999);
                }
            }
        }
    }
    /*
    int object_dis;
    int Dis_sm,Dis_bi,dis_num;
    double dis_ratio;
    for(int j=0;j<whiteline_pixel.size();j++){
        if(whiteline_pixel[j]==999){
            object_dis = 999;
        }else{
            Dis_sm = dis_pixel[0];
            Dis_bi = dis_pixel[0];
            if(whiteline_pixel[j]>dis_pixel[dis_pixel.size()-1]){
                object_dis = dis_space[dis_space.size()-1];
            }else{
                for(int i=1;i<dis_pixel.size();i++){
                    if(dis_pixel[i]<whiteline_pixel[j]){
                        dis_num = i;
                        Dis_sm = dis_pixel[i];
                        Dis_bi = dis_pixel[i];
                    }
                    if(dis_pixel[i]>whiteline_pixel[j]){
                        dis_num = i;
                        Dis_bi = dis_pixel[i];
                        break;
                    }
                }
                if(whiteline_pixel[j] == dis_pixel[dis_num-1]){
                    object_dis = dis_space[dis_num-1];
                }else if(whiteline_pixel[j] == dis_pixel[dis_num]){
                    object_dis = dis_space[dis_num];
                }else{
                    dis_ratio = (double)(whiteline_pixel[j]-Dis_sm)/(double)(Dis_bi-Dis_sm)*(double)dis_gap;
                    object_dis = dis_space[dis_num-1] + (int)dis_ratio;
                }
            }
        }
        WhiteRealDis.data.push_back(object_dis);
    }*/
    int object_dis;
    for(int j=0;j< whiteline_pixel.size();j++){
        object_dis = Omni_distance( whiteline_pixel[j]);
        WhiteRealDis.data.push_back(object_dis);
        //cout<<object_dis<<endl;
    }
    white_pub.publish(WhiteRealDis);
    ros::spinOnce();
    //////////////////////////////////////////////////////////b
    ////////////////////////////////////////////////////////////rrrrrrrrrrrrrrrr


    /////////////////////Show view/////////////////
//    cv::imshow("Image", frame);
//    cv::waitKey(10);
    ///////////////////////////////////////////////
    /////////////////////FPS///////////////////////
    int EndTime = ros::Time::now().toNSec();
    double fps = 1000000000/(EndTime - StartTime);
    //cout<<"FPS_avg : "<<fps<<endl;

    ///////////////////////////////////////////////
}

double ImageConverter::Omni_distance(double dis_pixel){
  double Z = -1*Camera_H;
  double c = 83.125;
  double b = c*0.8722;

  double f = Camera_f;

  double dis;
  //cout<<Z<<endl;
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
void ImageConverter::get_whitedata(){
    nh.getParam("/FIRA/whiteline/gray",white_gray);
    nh.getParam("/FIRA/whiteline/angle",white_angle);
}
void ImageConverter::get_Camera(){
    nh.getParam("/FIRA/Camera/High",Camera_H);
    nh.getParam("/FIRA/Camera/Focal",Camera_f);
}
