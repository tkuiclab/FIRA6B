/**
 * @file /QServer_server/QServer.hpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef QServer_NODE_HPP_
#define QServer_NODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include "../common/qnode.hpp"
#include <QObject>
#endif

#include <ros/ros.h>
#include "../common/qnode.hpp"
#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <vision/Object.h>
/*****************************************************************************
** Class
*****************************************************************************/
namespace enc = sensor_msgs::image_encodings;

Q_DECLARE_METATYPE(std::string);

class QInterface : public QNode {
 Q_OBJECT


public:
    QInterface(int argc, char** argv);
    virtual ~QInterface() {}
    void run();
    void ros_comms_init();
    std::string  ball_LR,blue_LR,yellow_LR;
    int image_fps;
    int   ball_x, ball_y, ball_ang, ball_dis;
    int   blue_x, blue_y,blue_ang, blue_dis;
    int   yellow_x, yellow_y, yellow_ang, yellow_dis;

    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void object_data(const vision::Object& msg);
    cv_bridge::CvImagePtr cv_ptr;
    //-------------------init_data----------------------
    void init_data(){
        ball_LR = "NULL";blue_LR = "NULL";yellow_LR = "NULL";
        image_fps = 999;
        ball_x = 999; ball_y = 999; ball_ang = 999; ball_dis = 999;
        blue_x = 999; blue_y = 999; blue_ang = 999; blue_dis = 999;
        yellow_x = 999; yellow_y = 999; yellow_ang = 999; yellow_dis = 999;
    }
    //--------------------------------------------------
    //-------------------camera-------------------------
    void set_campara(int value_ex){
        dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        dynamic_reconfigure::DoubleParameter double_param;
        dynamic_reconfigure::Config conf;
        double exposure = (double)value_ex/1000000;
        double_param.name = "exposure";
        double_param.value = exposure;
        conf.doubles.push_back(double_param);

        srv_req.config = conf;
        ros::service::call("/prosilica_driver/set_parameters", srv_req, srv_resp);
    }

    double camera_exposure;
    void get_campara(){
      camera_exposure = 0;
      nh->getParam("/prosilica_driver/exposure",camera_exposure);
    }

    //-------------------center-------------------------
    void sent_center(int center_x, int center_y, int center_inner, int center_outer, int center_front){
        nh->setParam("/FIRA/Center/X",center_x);
        nh->setParam("/FIRA/Center/Y",center_y);
        nh->setParam("/FIRA/Center/Inner",center_inner);
        nh->setParam("/FIRA/Center/Outer",center_outer);
        nh->setParam("/FIRA/Center/Front",center_front);
    }
    int center_get_x, center_get_y, center_get_inner, center_get_outer, center_get_front;
    void get_center(){
        center_get_x = 999;center_get_y = 999;
        center_get_inner = 999;center_get_outer = 999;center_get_front = 999;
        nh->getParam("/FIRA/Center/X",center_get_x);
        nh->getParam("/FIRA/Center/Y",center_get_y);
        nh->getParam("/FIRA/Center/Inner",center_get_inner);
        nh->getParam("/FIRA/Center/Outer",center_get_outer);
        nh->getParam("/FIRA/Center/Front",center_get_front);
    }
    //--------------------------------------------------
    //---------------------scan-------------------------
    void sent_scan(std::vector<int>scan_para){
        nh->setParam("/FIRA/Scan/Parameter",scan_para);
    }
    std::vector<int>scan_get_para;
    void get_scan(){
        scan_get_para.clear();
        nh->getParam("/FIRA/Scan/Parameter",scan_get_para);
    }
    //--------------------------------------------------
    //--------------------distance----------------------
    void sent_dis(int dis_gap,std::vector<int>dis_space, std::vector<int>dis_pixel){
        nh->setParam("/FIRA/Distance/Gap",dis_gap);
        nh->setParam("/FIRA/Distance/Space",dis_space);
        nh->setParam("/FIRA/Distance/Pixel",dis_pixel);
    }
    void sent_Camera(double High, double Focal){
        nh->setParam("/FIRA/Camera/High",High);
        nh->setParam("/FIRA/Camera/Focal",Focal);
    }
    int camera_High;
    void get_Camera(){
      camera_High = 0;
      nh->getParam("/FIRA/Camera/High",camera_High);
    }
    //--------------------------------------------------
    //----------------------HSV-------------------------
    void sent_hsv(std::vector<int>HSV_red, std::vector<int>HSV_green, std::vector<int>HSV_blue, std::vector<int>HSV_yellow, std::vector<int>HSV_white){
        nh->setParam("/FIRA/HSV/Redrange",HSV_red);
        nh->setParam("/FIRA/HSV/Greenrange",HSV_green);
        nh->setParam("/FIRA/HSV/Bluerange",HSV_blue);
        nh->setParam("/FIRA/HSV/Yellowrange",HSV_yellow);
        nh->setParam("/FIRA/HSV/Whiterange",HSV_white);
    }
    std::vector<int>Redmap,Greenmap,Bluemap,Yellowmap,Whitemap;
    void get_hsv(){
        Redmap.clear();Greenmap.clear();
        Bluemap.clear();Yellowmap.clear();
        Whitemap.clear();
        nh->getParam("/FIRA/HSV/Redrange",Redmap);
        nh->getParam("/FIRA/HSV/Greenrange",Greenmap);
        nh->getParam("/FIRA/HSV/Bluerange",Bluemap);
        nh->getParam("/FIRA/HSV/Yellowrange",Yellowmap);
        nh->getParam("/FIRA/HSV/Whiterange",Whitemap);
    }
    //--------------------------------------------------
    //----------------------whiteline-------------------
    void sent_whiteline(int gray, int angle){
        nh->setParam("/FIRA/whiteline/gray",gray);
        nh->setParam("/FIRA/whiteline/angle",angle);
    }
    //--------------------------------------------------
    //----------------------blackItem-------------------
    void sent_blackItem(int gray, int angle){
        nh->setParam("/FIRA/blackItem/gray",gray);
        nh->setParam("/FIRA/blackItem/angle",angle);
    }


    int black_gray,black_angle;
    void get_blackItem(){
      black_gray = 0;
      black_angle = 0;
      nh->getParam("/FIRA/blackItem/gray",black_gray);
      nh->getParam("/FIRA/blackItem/angle",black_angle);
    }
    //--------------------------------------------------

Q_SIGNALS:

private:
    ros::NodeHandle *nh;
    image_transport::ImageTransport *it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber object_sub;

};

#endif /* QServer_NODE_HPP_ */
