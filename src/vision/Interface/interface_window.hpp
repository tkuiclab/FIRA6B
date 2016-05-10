#ifndef INTERFACE_WINDOW_H
#define INTERFACE_WINDOW_H

#include <QMainWindow>
#include "Interface/ui_interface_window.h"
#include "qInterface.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QMouseEvent>
#include <QDebug>
#include <QEvent>

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

    //cv::Mat frame;
    int mosue_x,mosue_y;
    int distance_space[100];
    int distance_pixel[100];
    std::vector<int>scan_para;
    std::vector<int>scan_near;
    std::vector<int>scan_middle;
    std::vector<int>scan_far;
    std::vector<int>dis_space;
    std::vector<int>dis_pixel;
    int HSV_init[6];
    std::vector<int> HSV_red;
    std::vector<int> HSV_green;
    std::vector<int> HSV_blue;
    std::vector<int> HSV_yellow;

protected:
    void timerEvent(QTimerEvent *);

private Q_SLOTS:
    void opposite(cv::Mat);
    void Showimg(cv::Mat);
    void on_Slider_Exposure_valueChanged(int value);
    void on_Slider_White_R_valueChanged(int value);
    void on_Slider_White_B_valueChanged(int value);
    void Center(cv::Mat, int, int, int, int, int );
    void Scan(cv::Mat, int, int);
    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void dis_combox_and_list_renew();
    void on_Dis_space_valueChanged(int value);
    void on_Dis_num_valueChanged(int value);
    void Distance_draw(cv::Mat, int, int);
    void RGBtoHSV(cv::Mat, double *);
    void RGBtoHSV_maxmin(double &, double &, double &, double &, double &);
    double RGBtoHSV_H(double, double, double, double, double);
    double RGBtoHSV_S(double, double);
    void HSVtoRGB( double , double , double , int &, int &, int &);
    void on_HSV_comboBox_currentIndexChanged(int index);
    void HSVModel(cv::Mat, double *);
    void HSVmap();
    uchar * HSV_PrintAngle();
    uchar * HSV_PrintBackground();
    void Draw_inner_outer_circle(cv::Mat , int , int ,int , int );
    void White_Line(cv::Mat ,int, int , int ,int , int );
    void Black_Line(cv::Mat , int, int, int ,int , int );
    void Draw_Front_Line(cv::Mat , int , int ,int );
    void Data_check(cv::Mat ,int ,int);

Q_SIGNALS:
    void Mouse_pressed();
    
private:
    Ui::interface_window *ui;
    QInterface *interface;
};

#endif // INTERFACE_WINDOW_H
