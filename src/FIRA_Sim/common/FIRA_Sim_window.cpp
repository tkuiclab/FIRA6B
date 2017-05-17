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
#include "FIRA_Sim_window.hpp"
#include <QComboBox>
#include <QKeyEvent>
#define pi 3.14159

using namespace Qt;

int *whiteline_Dist;
Vector2i y_goal;
double robot_x;
double robot_y;
double robot_rotation;
double shift_x = 0;
double shift_y = 0;
double v_x;
double v_y;
imu accel;

/*****************************************************************************
** Implementation [ClientWindow]
*****************************************************************************/

ClientWindow::ClientWindow(QClient *node, QWidget *parent) :
    QMainWindow(parent),
    mClient(node),
    imgWidget(new ImageWidget),
    pfNum(500),
    sensorLineNum(90),
    mapW(884),
    mapH(644),
    testCount(0)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));

    setWindowTitle(QApplication::translate("ClientWindowDesign", mClient->nodeName().c_str(), 0, QApplication::UnicodeUTF8));

    /*********************
    ** Logging
    **********************/
    //ui.view_logging->setModel(mClient->loggingModel());
    //QObject::connect(mClient, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    qRegisterMetaType<std::vector<Vector2i> >();
    qRegisterMetaType<Vector2i>();
    QObject::connect(mClient, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(this, SIGNAL(paintRobot_signal(double,double,double,bool)), imgWidget, SLOT(paintRobot_slot(double,double,double,bool)));
    QObject::connect(this, SIGNAL(paintParticle_signal(std::vector<Vector2i> ,Vector2i)), imgWidget, SLOT(paintParticle_slot(std::vector<Vector2i>, Vector2i)));
    QObject::connect(this, SIGNAL(paintSensorLine_signal(Vector2i,std::vector<Vector2i>)), imgWidget, SLOT(paintSensorLine_slot(Vector2i,std::vector<Vector2i>)));
    QObject::connect(this, SIGNAL(paintImu_signal(Vector2i,Vector2i)),imgWidget, SLOT(paintImu_slot(Vector2i,Vector2i)));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
    imageObject.load("/home/iclab/FIRA_ws/devel/lib/FIRA_Sim/FIRA_Ground.png");
    imgWidget->setImgObj(imageObject);
    imgWidget->show();
    mapW_grid = mapW*resolution;
    mapH_grid = mapH*resolution;
    initPF();
    QFuture<void> updateInfoThread = QtConcurrent::run(this,&ClientWindow::updateInfo);
}

void ClientWindow::out_likeliHood_map_2Pic(){
    QImage newImg(mapW_grid,mapH_grid,QImage::Format_RGB888);

    QColor white(255,255,255);

    for(int i =0;i < mapH_grid;i++){
        for(int j =0;j < mapW_grid;j++){
            if(likeliHood_map[i*mapW_grid+j]){
                double tGray = (100.0-(double)likeliHood_map[i*mapW_grid+j])/100.0*255.0;
                QColor gray(tGray,tGray,tGray);
                newImg.setPixel(j,mapH_grid-i-1,gray.rgb());
                //newImg.setPixel(j,i,red.rgb());
            }else{
               newImg.setPixel(j,mapH_grid-i-1,white.rgb());
               //newImg.setPixel(j,i,white.rgb());
            }
        }
    }
    //for(int j =0;j < w;j++){
    //    newImg.setPixel(j,400,blue.rgb());
    //}

    newImg.save("likeliHood_map.png");

}

void ClientWindow::likeliHood_markWant(int wantIndex,int markNum){
    if( wantIndex> 0 && wantIndex < (mapW_grid*mapH_grid)){
        //maybe the map have marked already
        if(markNum > likeliHood_map[wantIndex]){
            likeliHood_map[wantIndex] = markNum;
        }
    }
}

void ClientWindow::likeliHood_surroundMark(int nowIndex,int dist,int markNum){
    int nowWant;
    //nowWant = nowIndex - dist*mapW -1;
    likeliHood_markWant(nowIndex - dist*mapW_grid -dist,markNum);
    likeliHood_markWant(nowIndex - dist*mapW_grid      ,markNum);
    likeliHood_markWant(nowIndex - dist*mapW_grid +dist,markNum);

    likeliHood_markWant(nowIndex             -dist,markNum);
    //likeliHood_markWant(nowIndex - dist*mapW +1,markNum);
    likeliHood_markWant(nowIndex             +dist,markNum);

    likeliHood_markWant(nowIndex + dist*mapW_grid -dist,markNum);
    likeliHood_markWant(nowIndex + dist*mapW_grid      ,markNum);
    likeliHood_markWant(nowIndex + dist*mapW_grid +dist,markNum);
}

void ClientWindow::initPF(){
    int i;
    v_x = 0;
    v_y = 0;
    map = new bool[mapW_grid*mapH_grid];
    sensorWall_Dist = new int[sensorLineNum];
    likeliHood_map = new int[mapW_grid*mapH_grid];
    for(i = 0 ;i < mapW_grid*mapH_grid;i++)   likeliHood_map[i] = 0;


    int r,g,b;
    //======generate map-bool&likeliHood-map array======
    //i,j is image position
    for(i =(imageObject.size().height()-1);i > 0;i--){
    //for(int i =0;i< mapH;i++){
        for(int j =0;j < mapW_grid;j++){
            QRgb trgb = imageObject.pixel(j,i);

            QColor(trgb).getRgb(&r,&g,&b);
            //black (0,0,0),white(255,255,255)
            if(r==0){
                int yMap = imageObject.size().height() - i;
                int tIndex =yMap * mapW_grid + j;
                map[tIndex] = true;
                //likeliHood_map[yMap * mapW + j] = 100;

                //map[i * mapW + j] = true;
                int markNum =100;
                int num = 5;
                //makr 100 to 10 (decrease by 10)
                for(int d =1;d<=num;d++){
                    //std::cout << "markNum=" << markNum << std::endl;
                    likeliHood_surroundMark(tIndex,d,markNum);
                    markNum-=markNum/num;

                }
            }
        }
    }
    out_likeliHood_map_2Pic();//output pic likeliHood_map.png

    //outMapBoolAry2Pic(mapW_grid,mapH_grid,map);

    pf.setMap(mapW,mapH,resolution,map);
    pf.initParticle(pfNum);
    pf.setSensorLineNum(sensorLineNum);
    pf.setLikeliHoodMap(likeliHood_map);
    //pf.echoPPos();
    //pf.moveParticle(Vector2i(1,2));
    //pf.echoPPos();
    whiteline_Dist = this->mClient->return_whiteline();
    Vector2i rPosi;
    rPosi(0) = 442;
    rPosi(1) = 322;
    sensorWall_relPose.clear();
    for(int i=0;i<sensorLineNum;i++)
    {
        double eachDeg = 360/sensorLineNum;
        double to_rad = i*eachDeg*2*pi/360;
        double nowDeg = robot_rotation + to_rad;
        if(nowDeg >= 2*pi)
        {
            nowDeg = nowDeg - 2*pi;
        }
        Vector2i tmp;
        tmp(0) = whiteline_Dist[i]*cos(nowDeg)/100*117.87;
        tmp(1) = whiteline_Dist[i]*sin(nowDeg)/100*117.87*-1;
        sensorWall_relPose.push_back(tmp);
    }
    //Q_EMIT paintSensorLine_signal(rPosi,sensorWall_relPose);
    Q_EMIT paintParticle_signal(pf.getPosAry_grid(),pf.getPredictionPos_grid());
}

//void ClientWindow::sensorWall(Vector2i robotPos, double robotRot){
//    double nowDeg = 0;
//    double eachDeg = (360.0/sensorLineNum);
//    double nowRad;
//    sensorWall_relPose.clear();
//    //printf("Out nowDeg=%f\n",nowDeg);
//    //printf("eachDeg=%f\n",eachDeg);


//    for(int i = 0;i < sensorLineNum;i++){
//        nowDeg = i * eachDeg;           //ex: 0,4,8...356
//        //nowRad = deg2rad(nowDeg);
//        nowRad = nowDeg*2*pi/360;

//        //printf("=================i=%d================\n",i);
//        //printf("eachDeg=%f\n",eachDeg);
//        //printf("NowDeg=%f\n",nowDeg);
//        for(int j=1;;j++){
//            Vector2d distPos(j,0);
//            Rotation2Dd rot(nowRad);
//            Vector2d addPos = rot * distPos;
//            Vector2i addPosi = Vector2i(addPos(0),addPos(1));
//            Vector2i newPos = robotPos + addPosi;

//            int index = newPos(1)*mapW_grid + newPos(0);

//            if(map[index]){
//                sensorWall_relPose.push_back(addPosi);
//                sensorWall_Dist[i]=j;

//                break;
//            }

//            if(newPos(0) < 0 || newPos(0) >= mapW_grid
//                        || newPos(1) < 0 || newPos(1) >= mapH_grid)
//            {
//                            addPosi(0) = 999;addPosi(1) = 999;
//                            sensorWall_relPose.push_back(addPosi);
//                            sensorWall_Dist[i] = 999;//not detcet
//                            break;
//            }
//        }

//    }

//    //for(int i = 0;i < sensorLineNum;i++){
//    //    std::cout << "sensor[" << i << "]=" << sensorWall_Dist[i] << std::endl;
//    //}
//    /*
//    std::cout << "=====in sensorWall()====";
//    for(int j=0;j < sensorWall_relPose.size();j++){
//        std::cout << "sensorWall_relPose[j](1)=" << sensorWall_relPose[j](1) << std::endl;
//        std::cout << "sensorWall_relPose[j](0)=" << sensorWall_relPose[j](0) << std::endl;
//    }
//    */


//}

double ClientWindow::rad2deg(double inVal){
    return 180/M_PI*inVal;

}


double ClientWindow::deg2rad(double inVal){
    return M_PI/180*inVal;

}

void ClientWindow::updateInfo() {
    robot_x = 442;
    robot_y = 322;

    Vector2d rPos( robot_x,robot_y);
    Vector2d prePos( robot_x,robot_y);




    bool saveNow = true;

    double sumTime = 0;

    int iterationTime = 0;

    while(true){
        try {
            //get robot position
            accel = this->mClient->return_accel();
            whiteline_Dist = this->mClient->return_whiteline();
            y_goal = this->mClient->return_doorDist();
            //printf("%d\n",whiteline_Dist[1]);
            //printf("%d\t%d\n",y_goal(0),y_goal(1));

            robot_x = robot_x + accel.shift_x*117.87;
            robot_y = robot_y + accel.shift_y*117.87;
            robot_rotation = accel.rotation;

            //printf("%f\n",(robot_x-442)/117.87);
            //printf("%f\n",(robot_y-322)/117.87);
            //printf("%f\n",robot_rotation);

            rPos(0) = robot_x;
            rPos(1) = robot_y;


            Vector2i rPosi(rPos(0),rPos(1));
            Vector2i prePosi(prePos(0),prePos(1));
            Vector2d subPos = rPos - prePos;
            //std::cout << "rPos=" << rPos << std::endl;
            //std::cout << "subPos=" << subPos << std::endl;

            //if(sqrt(subPos(0)*subPos(0)+subPos(1)*subPos(1)) > 0.3){
            double tDist =hypot(subPos(0),subPos(1));
            if( tDist > 10){
                //pfCount++;
                this->pf.getrobotpos(robot_x,robot_y);
                iterationTime++;

                //std::cout << "<======iterationTime=======>\n iterationTime:" << iterationTime << std::endl;

                double CanNotConvergence = sqrt((pf.predictionPos(0)-robot_x)*(pf.predictionPos(0)-robot_x) + (pf.predictionPos(1)-robot_y)*(pf.predictionPos(1)-robot_y));

                if(iterationTime > 10 && CanNotConvergence > 40 )
                {
                    CanNotConvergence = 0;
                    iterationTime = 0;
                    pf.initParticle(pfNum);
                }

                //Calculate sensor wall position
                sensorWall_relPose.clear();
                for(int i=0;i<sensorLineNum;i++)
                {
                    double eachDeg = 360/sensorLineNum;
                    double to_rad = i*eachDeg*2*pi/360;
                    double nowDeg = robot_rotation + to_rad;
                    if(nowDeg >= 2*pi)
                    {
                        nowDeg = nowDeg - 2*pi;
                    }
                    Vector2i tmp;
                    tmp(0) = whiteline_Dist[i]*cos(nowDeg)/100*117.87;
                    tmp(1) = whiteline_Dist[i]*sin(nowDeg)/100*117.87*-1;
                    sensorWall_relPose.push_back(tmp);
                }

                //open sensor!!get wall distance
                //sensorWall(rPosi, robot_rotation);
                prePos = rPos;

                //about pf
                //particle move
                pf.moveParticle(subPos);
                pf.rateGrade(sensorWall_relPose);
                pf.prediction();
                pf.rePickParticle();

                saveNow = true;
                Q_EMIT paintParticle_signal(pf.getPosAry_grid(),pf.getPredictionPos_grid());
                Q_EMIT paintSensorLine_signal(rPosi,sensorWall_relPose);
            }



            //get robot angle
            double yaw = robot_rotation; //Yaw

            //update robot infomation to ui


            //imgWidget->drawPoint(x,y);
            Q_EMIT paintRobot_signal(robot_x,robot_y,accel.rotation,saveNow);
            if(saveNow){
                saveNow = false;

            }
            Q_EMIT paintImu_signal(rPosi,prePosi);

            //QThread::msleep(50);

            //sleep for 50ms
            QWaitCondition wc;
            QMutex mutex;
            QMutexLocker locker(&mutex);
            wc.wait(&mutex, 10);

        } catch (std::exception &e) {
            //qFatal("Error %s sending event %s to object %s (%s)",
           //     e.what(), typeid(*event).name(), qPrintable(receiver->objectName()),
            //    typeid(*receiver).name());
            std::cout << "ClientWindow::updateInfo() error" << e.what() << std::endl;
        } catch (...) {
            std::cout << "ClientWindow::updateInfo() error" << std::endl;
        }
    }
}

ClientWindow::~ClientWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void ClientWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void ClientWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !mClient->on_init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
            ui.rightBtn->setEnabled(true);
            ui.leftBtn->setEnabled(true);
            ui.downBtn->setEnabled(true);
            ui.upBtn->setEnabled(true);
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

void ClientWindow::on_checkbox_use_environment_stateChanged(int state) {
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
//void ClientWindow::updateLoggingView() {
//        ui.view_logging->scrollToBottom();
//}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void ClientWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void ClientWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", mClient->nodeName().c_str());
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    }
}
void ClientWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", mClient->nodeName().c_str());
    settings.setValue("geometry", geometry());
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
   	settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
}

void ClientWindow::closeEvent(QCloseEvent *event)
{
    mClient->shutdown();
	WriteSettings();
    QMainWindow::closeEvent(event);
}
