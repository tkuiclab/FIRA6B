#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <deque>


#include "std_msgs/Int32MultiArray.h"

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;

class ImageConverter
{
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher black_pub;
    std_msgs::Int32MultiArray BlackRealDis;

    int center_x, center_y, center_inner, center_outer, center_front;
    int dis_gap;
    std::vector<int>dis_space, dis_pixel;
    int black_gray,black_angle;

    std::vector<double>blackItem_pixel;

    Mat Main_frame;

    int frame_counter;
    long int EndTime;
    long int dt;

    double Camera_H,Camera_f;

    std::vector<double> Angle_sin;
    std::vector<double> Angle_cos;


public:
    ImageConverter();
    ~ImageConverter();

    double Omni_distance(double dis_pixel);
    void imageCb(const sensor_msgs::ImageConstPtr&);
    void get_center();
    void get_distance();
    void get_whitedata();
    void get_Camera();
};

