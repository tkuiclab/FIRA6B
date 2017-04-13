#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <deque>
#include <vision/Object.h>
#include <ros/package.h>

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
typedef unsigned char BYTE;

class object_Item{
  public:
  int dis_max;
  int dis_min;
  int ang_max;
  int ang_min;
  int x;
  int y;
  int angle;
  double distance;
  int size;
  string LR;
};

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

    string Red_LR,Blue_LR,Yellow_LR;
    int Red_angle,Blue_angle,Yellow_angle;
    int Red_dis,Blue_dis,Yellow_dis;

    ////FPS////
    int frame_counter;
    long int EndTime;
    long int dt;

    Mat Main_frame;
    Mat Findmap;
    Mat FIRA_map;
    Mat Obstaclemap;
    Mat Erodemap;
    Mat Dilatemap;

    int search_angle;
    int search_distance;
    int search_start;
    int search_near;
    int search_middle;
    int search_end;

    int dont_angle[6];

    vector<double> Angle_sin;
    vector<double> Angle_cos;

    deque<int> find_point;

    object_Item FIND_Item,Red_Item,Yellow_Item,Blue_Item;
    object_Item *Obstacle_Item;

    int Camera_H;
    double Camera_f;

    std::string vision_path;

public:
    ImageConverter();
    ~ImageConverter();

    void imageCb(const sensor_msgs::ImageConstPtr&);
    vector<BYTE> ColorFile();
    void get_center();
    void get_scan();
    void get_Camera();

    int angle_for_distance(int dis);
    void objectdet_change(Mat &, int, object_Item &);
    void creat_Obstclemap(Mat &, int);
    void creat_FIRA_map(Mat &, Mat &);

    void objectdet_Obstacle(Mat &, int, object_Item *);
    void Mark_point(Mat &, int, int, int, int, int &, int);
    void object_Item_reset(object_Item &);
    void find_around(Mat &, int, int, int &, int);
    void object_compare(int, int);
    void draw_ellipse(Mat &, object_Item &);
    void draw_Line(Mat &, int, int, int);

    void find_object_point(object_Item &, int);

    double Omni_distance(double);
};

