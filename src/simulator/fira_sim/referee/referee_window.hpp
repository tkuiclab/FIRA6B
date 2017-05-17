/**
 * @file common/main_window.hpp
 *
 * @brief Qt based gui for eros_qQClient.
 *
 * @date November 2010
 **/
#ifndef QTUTORIALS_SERVER_WINDOW_H
#define QTUTORIALS_SERVER_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>

#ifndef Q_MOC_RUN
#include "referee/ui_referee_window.h"
#include "qClient.hpp"
#endif


#define GameState_Play          0
#define GameState_Halt          1
#define GameState_FreeKick      2
#define GameState_PenaltyKick   3
#define GameState_Kickoff      4
#define GameState_ThrowIn       5
#define GameState_CornerKick    6
#define GameState_GoalKick      7


#include "connect_ros_window.hpp"
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class RefereeWindow : public QMainWindow {
Q_OBJECT

public:
    RefereeWindow(QClient *node, QWidget *parent = 0);
    ~RefereeWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
    /******************************************
    ** Manual connections
    *******************************************/

    void on_actionConnect_ROS_Setting_triggered(){connRosWin->show();}

    void on_resetWorldBtn_clicked();
    void on_getGoal_clicked();
    void sendgetGoal();


private:
    Ui::RefereeWindowDesign ui;
    //QNode *qnode;
    QClient *mClient;

    ConnectRosWindow *connRosWin;
};

#endif // QTUTORIALS_MAIN_WINDOW_H
