#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <deque>
#include <vision/Object.h>

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
typedef unsigned char BYTE;

class ImageConverter
{
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher object_pub;

    vector<BYTE> color_map;
    int center_x, center_y, center_inner, center_outer, center_front;
    vector<int> scan_para, scan_near, scan_middle, scan_far;
    int dis_gap;
    std::vector<int>dis_space, dis_pixel;
    vector<int> near_num,near_point,middle_num,middle_point,far_num,far_point;
    deque<int> near_last,middle_last,far_last;
    vector<int> near_last_num,middle_last_num,far_last_num;

    string Red_LR,Blue_LR,Yellow_LR;
    int Red_angle,Blue_angle,Yellow_angle;
    int Red_dis,Blue_dis,Yellow_dis;

    int num;

    int core_num;
    int fps_num[100];
    int fps_avg;

public:
    ImageConverter();
    ~ImageConverter();

    void imageCb(const sensor_msgs::ImageConstPtr&);
    vector<BYTE> ColorFile();
    void get_center();
    void get_scan();
    void get_distance();
    void opposite(cv::Mat);
    void HSVmap(Mat, Mat, Mat, Mat, Mat);
    void objectdet(Mat, int &, int &, int &, int &);
    void objectdet_search(Mat , vector<int> &,deque<int> &, int , int, int );
    void objectdet_point(Mat, vector<int> &,deque<int> &, int, int, int, int );
    void objectdet_arund(Mat, vector<int> &,deque<int> &,int , int , int, int, int );
    void objectdet_color(Mat, Mat, Mat, Mat, Mat );
    void draw(Mat, int, int, int, int);
    void place_case(Mat, int &, int &, int &, int &);
    void objectdet_distance(int, int, string &, int &, int &);
};

