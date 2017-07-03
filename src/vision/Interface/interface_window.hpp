#ifndef INTERFACE_WINDOW_H
#define INTERFACE_WINDOW_H

#include <QMainWindow>
#include "Interface/ui_interface_window.h"
#include "qInterface.hpp"
#include <algorithm>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QMouseEvent>
#include <QDebug>
#include <QEvent>
#include <QPainter>
#include <QPixmap>

#include <ros/package.h>

using namespace cv;
namespace Ui {
class interface_window;
}

class interface_window : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit interface_window(QInterface *node, QWidget *parent = 0);
    ~interface_window();

    int mosue_x,mosue_y;

    std::vector<int>scan_para;
    int HSV_init[6];
    std::vector<int> HSV_red;
    std::vector<int> HSV_green;
    std::vector<int> HSV_blue;
    std::vector<int> HSV_yellow;
    std::vector<int> HSV_white;

    int frame_counter;
    long int EndTime;
    long int dt;

    Mat Main_frame;
    QPixmap dest;

    int center_x, center_y, center_inner, center_outer, center_front;
    int camera_high;
    double camera_focal;

    int search_angle;
    int search_distance;
    int search_start;
    int search_near;
    int search_middle;
    int search_end;

    std::vector<double> Angle_sin;
    std::vector<double> Angle_cos;

    int dont_angle[6];

    std::string vision_path;
    std::string fira_launch_path;

protected:
    void timerEvent(QTimerEvent *);

private Q_SLOTS:
    void Showimg(QPixmap);
    QImage cvMatToQImage( const cv::Mat &inMat );
    QPixmap cvMatToQPixmap( const cv::Mat &inMat);

    void Center(Mat &);

    int angle_for_distance(int);
    void Scan(cv::Mat &);

    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);

    void RGBtoHSV(cv::Mat &, double *);
    void RGBtoHSV_maxmin(double &, double &, double &, double &, double &);
    double RGBtoHSV_H(double, double, double, double, double);
    double RGBtoHSV_S(double, double);
    void HSVtoRGB( double , double , double , int &, int &, int &);
    void on_HSV_comboBox_currentIndexChanged(int index);
    void HSVModel(cv::Mat &, double *);
    void HSVmap();
    uchar * HSV_PrintAngle();
    uchar * HSV_PrintBackground();

    void White_Line(cv::Mat &);
    void Black_Line(cv::Mat &);

    void Data_check(cv::Mat &);
    void Draw_cross(cv::Mat &,char);

    double camera_f(int);
    double Omni_distance(int x , int y);

    void on_Exposure_send_clicked();


    void on_Parameter_Dump_clicked();

Q_SIGNALS:
    void Mouse_pressed();
    
private:
    Ui::interface_window *ui;
    QInterface *interface;
};

#endif // INTERFACE_WINDOW_H
