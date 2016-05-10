#include "image_converter.hpp"
#include "math.h"

#define IMAGE_TEST1 "src/vision/1.bmp"

using namespace std;
using namespace cv;

ImageConverter::ImageConverter()
   :it_(nh)
{
    get_center();
    get_distance();
    get_whitedata();
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
    black_pub  = nh.advertise<std_msgs::Int32MultiArray>("/vision/BlackRealDis",1);
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
    //cv_ptr->image = imread( IMAGE_TEST1 , CV_LOAD_IMAGE_COLOR );
    opposite(cv_ptr->image);
    double gray_num[256] = {0};
    int avg_new=0;
    int gray_sum = 0;
    int gray_avg = 0;
    int gray_low = 0;
    int gray_hight = 0;
    int avg_old = 0;
    for(int i=0;i<cv_ptr->image.rows;i++){
        for(int j=0;j<cv_ptr->image.cols;j++){
            unsigned char gray =(cv_ptr->image.data[(i*cv_ptr->image.cols*3)+(j*3)+0]+
                                cv_ptr->image.data[(i*cv_ptr->image.cols*3)+(j*3)+1]+
                                cv_ptr->image.data[(i*cv_ptr->image.cols*3)+(j*3)+2])/3;
            cv_ptr->image.data[(i*cv_ptr->image.cols*3)+(j*3)+0] = gray;
            cv_ptr->image.data[(i*cv_ptr->image.cols*3)+(j*3)+1] = gray;
            cv_ptr->image.data[(i*cv_ptr->image.cols*3)+(j*3)+2] = gray;
            gray_num[gray]++;
        }
    }
    for(int i=0;i<256;i++)gray_sum = gray_sum+(gray_num[i]*i);
    gray_avg = gray_sum/(cv_ptr->image.rows*cv_ptr->image.cols);
    avg_new = gray_avg;
    while((avg_old!=avg_new)&&(avg_old!=avg_new-1)){
        avg_old = avg_new;
        for(int i=0;i<avg_old;i++)gray_low = gray_low+(gray_num[i]*i);
        gray_low = gray_low/(cv_ptr->image.rows*cv_ptr->image.cols);
        for(int i=avg_old;i<256;i++)gray_hight = gray_hight+(gray_num[i]*i);
        gray_hight = gray_hight/(cv_ptr->image.rows*cv_ptr->image.cols);
        avg_new = (gray_low+gray_hight)/2;
    }
    for(int i=0;i<cv_ptr->image.rows;i++){
        for(int j=0;j<cv_ptr->image.cols;j++){
            if(cv_ptr->image.data[(i*cv_ptr->image.cols*3)+(j*3)+0]<avg_new){
                cv_ptr->image.data[(i*cv_ptr->image.cols*3)+(j*3)+0] = 0;
                cv_ptr->image.data[(i*cv_ptr->image.cols*3)+(j*3)+1] = 0;
                cv_ptr->image.data[(i*cv_ptr->image.cols*3)+(j*3)+2] = 0;
            }else{
                cv_ptr->image.data[(i*cv_ptr->image.cols*3)+(j*3)+0] = 255;
                cv_ptr->image.data[(i*cv_ptr->image.cols*3)+(j*3)+1] = 255;
                cv_ptr->image.data[(i*cv_ptr->image.cols*3)+(j*3)+2] = 255;
            }
        }
    }
    blackItem_pixel.clear();
    BlackRealDis.data.clear();
    for(int a=0;a<360;a=a+black_angle){
        int angle_be = a+center_front;
        if(angle_be>360)angle_be-=360;
        double angle_af = angle_be*M_PI/180;
        double x = cos(angle_af);
        double y = sin(angle_af);
        for(int r=center_inner;r<=center_outer;r++){
            int dis_x = x*r;
            int dis_y = y*r;
            if( cv_ptr->image.data[((center_y-dis_y)*cv_ptr->image.cols*3)+((center_x+dis_x)*3)+0] == 0
              &&cv_ptr->image.data[((center_y-dis_y)*cv_ptr->image.cols*3)+((center_x+dis_x)*3)+1] == 0
              &&cv_ptr->image.data[((center_y-dis_y)*cv_ptr->image.cols*3)+((center_x+dis_x)*3)+2] == 0){
                blackItem_pixel.push_back(hypot(dis_x,dis_y));
		break;
            }else{
//                cv_ptr->image.data[((center_y-dis_y)*cv_ptr->image.cols*3)+((center_x+dis_x)*3)+0] = 0;
//                cv_ptr->image.data[((center_y-dis_y)*cv_ptr->image.cols*3)+((center_x+dis_x)*3)+1] = 0;
//                cv_ptr->image.data[((center_y-dis_y)*cv_ptr->image.cols*3)+((center_x+dis_x)*3)+2] = 255;
                if(r==center_outer){
                    blackItem_pixel.push_back(999);
                }
            }
        }
    }
    int object_dis;
        int Dis_sm,Dis_bi,dis_num;
        double dis_ratio;
        for(int j=0;j<blackItem_pixel.size();j++){
            if(blackItem_pixel[j]==999){
                object_dis = 999;
            }else{
                Dis_sm = dis_pixel[0];
                Dis_bi = dis_pixel[0];
                if(blackItem_pixel[j]>dis_pixel[dis_pixel.size()-1]){
                    object_dis = dis_space[dis_space.size()-1];
                }else{
                    for(int i=1;i<dis_pixel.size();i++){
                        if(dis_pixel[i]<=blackItem_pixel[j]){
                            dis_num = i;
                            Dis_sm = dis_pixel[i];
                            Dis_bi = dis_pixel[i];
                        }
                        if(dis_pixel[i]>blackItem_pixel[j]){
                            dis_num = i;
                            Dis_bi = dis_pixel[i];
                            break;
                        }
                    }
                    if(blackItem_pixel[j] == dis_pixel[dis_num-1]){
                        object_dis = dis_space[dis_num-1];
                    }else if(blackItem_pixel[j] == dis_pixel[dis_num]){
                        object_dis = dis_space[dis_num];
                    }else{
                        dis_ratio = (double)(blackItem_pixel[j]-Dis_sm)/(double)(Dis_bi-Dis_sm)*(double)dis_gap;
                        object_dis = dis_space[dis_num-1] + (int)dis_ratio;
                    }
                }
            }
            BlackRealDis.data.push_back(object_dis);
        }
        black_pub.publish(BlackRealDis);
        ros::spinOnce();
    /////////////////////Show view/////////////////
//    cv::imshow("Image", cv_ptr->image);
//    cv::waitKey(10);
    ///////////////////////////////////////////////
}
void ImageConverter::opposite(Mat frame){
    Mat Outing(Size(frame.cols,frame.rows),CV_8UC3);
    for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
            Outing.data[(i*Outing.cols*3)+(j*3)+0] = frame.data[(i*frame.cols*3)+((frame.cols-j-1)*3)+0];
            Outing.data[(i*Outing.cols*3)+(j*3)+1] = frame.data[(i*frame.cols*3)+((frame.cols-j-1)*3)+1];
            Outing.data[(i*Outing.cols*3)+(j*3)+2] = frame.data[(i*frame.cols*3)+((frame.cols-j-1)*3)+2];
        }
    }
    for(int i=0;i<frame.rows*frame.cols*3;i++)frame.data[i] = Outing.data[i];
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
    nh.getParam("/FIRA/blackItem/gray",black_gray);
    nh.getParam("/FIRA/blackItem/angle",black_angle);
}
