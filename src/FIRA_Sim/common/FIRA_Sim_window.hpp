/**
 * @file common/main_window.hpp
 *
 * @brief Qt based gui for eros_qQClient.
 *
 * @date November 2010
 **/
#ifndef CLIENT_WINDOW_H
#define CLIENT_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include <QKeyEvent>

#ifndef Q_MOC_RUN
#include "common/ui_FIRA_Sim_window.h"
//#include "qnode.hpp"
#include <eigen3/Eigen/Dense>
#include "../movetest/qClient.hpp"
#include "../movetest/imageWidget.hpp"
#include "../ParticleFilter/ParticleFilter.hpp"
#include <QTimer>
#endif


using namespace Eigen;


 Q_DECLARE_METATYPE(std::vector<Vector2i>);
 Q_DECLARE_METATYPE(Vector2i);
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class ClientWindow : public QMainWindow {
Q_OBJECT

public:
    ClientWindow(QClient *node, QWidget *parent = 0);
    ~ClientWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

    void updateInfo();
public Q_SLOTS:
	// Put automatically triggered slots here (because of connectSlotsByName())
	// void on_button_enable_clicked(bool check); // example only
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/

private Q_SLOTS:

Q_SIGNALS:
    void paintRobot_signal(double,double,double,bool);
    void paintParticle_signal(std::vector<Vector2i>,Vector2i);
    void paintSensorLine_signal(Vector2i,std::vector<Vector2i> );
    void paintImu_signal(Vector2i,Vector2i);


private:

    Ui::GazeboControlWindowDesign ui;
    //QNode *qnode;
    QClient *mClient;

    ImageWidget *imgWidget;
    QImage imageObject;

    double rad2deg(double inVal);
    double deg2rad(double inVal);

    //start----about PF------

    ParticleFilter pf;
    int pfNum;
    double mapH,mapW;      //10cm/unit
    int mapW_grid,mapH_grid;

    bool* map;
    int*  likeliHood_map;
    static const int resolution = 1;  //1 meter(grid) 1 pixel


    int sensorLineNum;
    int *sensorWall_Dist; //size same as sensorLineNum
    std::vector<Vector2i> sensorWall_relPose;//"relative" wall postion from robot!!size same as sensorLineNum
    //note:
    //  sensorWall_Dist: distance between robot & wall from every angle
    //  sensorWall_relPose: the wall position from every angel(relative from robot);


    void initPF();
    void sensorWall(Vector2i robotPos, double robotRot);

    static double velocity_x(double t,double accel_x);
    static double velocity_y(double t,double accel_y);
    double shift(double (*fx)(double,double), double accel, double t);


    //mark likeliHood_map makr wantIndex with markNum
    void likeliHood_markWant(int wantIndex,int markNum);
    //mark likeliHood_map surround mark
    void likeliHood_surroundMark(int nowIndex,int dist,int markNum);


    //end----about PF------

    //for output
    void outMapBoolAry2Pic(int w,int h,bool* ary);
    void out_likeliHood_map_2Pic();


    int testCount;
};
#endif // QTUTORIALS_MAIN_WINDOW_H
