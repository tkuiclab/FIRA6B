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
#include "connect_ros_window.hpp"


using namespace Qt;

/*****************************************************************************
** Implementation [ConnectRosWindow]
*****************************************************************************/

ConnectRosWindow::ConnectRosWindow(QClient *node, QWidget *parent) :
    QMainWindow(parent),
    mClient(node)
{
    std::cout << "ConnectRosWindow::ConnectRosWindow() happen" << std::endl;

	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    //QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
    //ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

    setWindowTitle(QApplication::translate("ConnectRosWindowDesign", mClient->nodeName().c_str(), 0, QApplication::UnicodeUTF8));

    /*********************
    ** Logging
    **********************/
    //ui.view_logging->setModel(mClient->loggingModel());
    //QObject::connect(mClient, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(mClient, SIGNAL(rosShutdown()), this, SLOT(close()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_connectROSOnStartup->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

ConnectRosWindow::~ConnectRosWindow() {
    std::cout << "ConnectRosWindow::~~~ConnectRosWindow() happen" << std::endl;
    mClient->shutdown();
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void ConnectRosWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void ConnectRosWindow::on_button_connect_clicked(bool check ) {


	if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !mClient->on_init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
        }
	} else {
        if ( ! mClient->on_init(
					ui.line_edit_master->text().toStdString(),
					ui.line_edit_host->text().toStdString() )
				) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
		}
	}
}

void ConnectRosWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
//void ConnectRosWindow::updateLoggingView() {
//        ui.view_logging->scrollToBottom();
//}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

//void ConnectRosWindow::on_actionAbout_triggered() {
//    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));/
//}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void ConnectRosWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", mClient->nodeName().c_str());
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);

    //checkbox remember_settting
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    //checkbox_connectROSOnStartup
    bool connectROSOnStartup = settings.value("connectROSOnStartup", false).toBool();
    ui.checkbox_connectROSOnStartup->setChecked(connectROSOnStartup);
    //checkbox_use_environment
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    }
}

void ConnectRosWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", mClient->nodeName().c_str());
    settings.setValue("geometry", geometry());
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
   	settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
    settings.setValue("connectROSOnStartup",QVariant(ui.checkbox_connectROSOnStartup->isChecked()));
}

void ConnectRosWindow::closeEvent(QCloseEvent *event)
{
    std::cout << "ConnectRosWindow::closeEvent happen" << std::endl;

	WriteSettings();
    QMainWindow::closeEvent(event);
}


void ConnectRosWindow::on_quit_button_clicked()
{
    this->hide();
}

