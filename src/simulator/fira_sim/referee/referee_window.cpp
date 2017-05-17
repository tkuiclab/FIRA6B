/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "referee_window.hpp"
#include "std_msgs/String.h"
#include "ros/ros.h"


using namespace Qt;

/*****************************************************************************
** Implementation [RefereeWindow]
*****************************************************************************/

RefereeWindow::RefereeWindow(QClient *node, QWidget *parent) :
    QMainWindow(parent),
    mClient(node)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

    setWindowTitle(QApplication::translate("Referee Window", mClient->nodeName().c_str(), 0, QApplication::UnicodeUTF8));


     QObject::connect(mClient, SIGNAL(rosShutdown()), this, SLOT(close()));


    connRosWin = new ConnectRosWindow(node,parent);

    //***TIME COUNTER

}

RefereeWindow::~RefereeWindow() {}


void RefereeWindow::sendgetGoal(){
    mClient->sendStr("Info");

    QString bg = QString::number(mClient->Blue_G);
    QString yg = QString::number(mClient->Yellow_G);
    int gs = mClient->Game_S;
    std::string wb = mClient->Who_B;

    ui.Blue_Lab->setText("Blue_Score : "+bg);
    ui.Yellow_Lab->setText("Yellow_Score : "+yg);


    if(gs == GameState_Play)             ui.State_Lab->setText("State : Play");
    else if(gs == GameState_Halt)        ui.State_Lab->setText("State : Halt");
    else if(gs == GameState_FreeKick)    ui.State_Lab->setText("State : FreeKick");
    else if(gs == GameState_PenaltyKick) ui.State_Lab->setText("State : PenaltyKick");
    else if(gs == GameState_Kickoff)     ui.State_Lab->setText("State : Kickoff");
    else if(gs == GameState_ThrowIn)     ui.State_Lab->setText("State : ThrowIn");
    else if(gs == GameState_CornerKick)  ui.State_Lab->setText("State : CornerKick");
    else if(gs == GameState_GoalKick)    ui.State_Lab->setText("State : GoalKick");

    ui.Whos_Lab->setText("Player : "+QString::fromStdString(wb));

}

void RefereeWindow::closeEvent(QCloseEvent *event)
{
    std::cout << "RefereeWindow::closeEvent happen" << std::endl;
    mClient->shutdown();
    QMainWindow::closeEvent(event);
}


void RefereeWindow::on_resetWorldBtn_clicked()
{
    mClient->sendStr("Reset");

}

void RefereeWindow::on_getGoal_clicked()
{
    //QTimer::singleShot(200, this, SLOT(sendgetGoal()));

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(sendgetGoal()));
    timer->start(50);



//    mClient->sendStr("Goal");

//    QString bg = QString::number(mClient->Blue_G);
//    QString yg = QString::number(mClient->Yellow_G);
//    ui.Blue_Lab->setText("Blue_Score : "+bg);
//    ui.Yellow_Lab->setText("Yellow_Score : "+yg);
}
