/**
 * @file connect_ros_window.hpp
 *
 * @brief Qt based gui for eros_qQClient.
 *
 * @date November 2010
 * @author Kartik Chen
 * @version 1.0
 * @update 2014/08/03    Version 1.0
 **/
#ifndef QTUTORIALS_CONNECT_ROS_WINDOW_H
#define QTUTORIALS_CONNECT_ROS_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>

#ifndef Q_MOC_RUN
#include "referee/ui_connect_ros_window.h"
#include "qClient.hpp"
#endif

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class ConnectRosWindow : public QMainWindow {
Q_OBJECT

public:
    ConnectRosWindow(QClient *node, QWidget *parent = 0);
    ~ConnectRosWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

    std::string getHost(){return ui.line_edit_host->text().toStdString();}
    std::string getMaster(){return ui.line_edit_master->text().toStdString();}
    bool getStartupConnect(){return ui.checkbox_connectROSOnStartup->isChecked();}

public Q_SLOTS:
	// Put automatically triggered slots here (because of connectSlotsByName())
	// void on_button_enable_clicked(bool check); // example only
    //void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);


    void on_quit_button_clicked();

    //void on_quit_button_clicked(bool checked);

private:

    Ui::ConnectROSWindowDesign ui;
    //QNode *qnode;
    QClient *mClient;
};

#endif // QTUTORIALS_MAIN_WINDOW_H
