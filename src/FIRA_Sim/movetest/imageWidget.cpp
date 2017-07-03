#include "imageWidget.hpp"

ImageWidget::ImageWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ImageWidget),
    imgObj_saveIndex(0)
{
    ui->setupUi(this);

    QApplication::setOverrideCursor(Qt::CrossCursor);
    setMouseTracking(true); // E.g. set in your constructor of your widget.

}

ImageWidget::~ImageWidget()
{
    delete ui;
}

//void ImageWidget::mouseMoveEvent(QMouseEvent *event){
//    qDebug() << event->pos();
//}

void ImageWidget::paintRobot_slot(double x,double y,double yaw,bool saveImg){

    int halfSize = 4;    //機器人再圖上的半徑
    int rawLineSize = 10;//車頭方向


    //imgObj = oriImg.copy();
    imgObj = particleImg.copy();
    QPainter painter (&imgObj);
    painter.setPen(Qt::red);
    painter.setBrush(Qt::red);   //fill Circle


   // std::cout << "in X=" << x <<",in Y=" << y << std::endl;


    //std::cout << "in paintX=" << paintX <<",in paintY=" << paintY<< std::endl;

    //double tmp = 45*(PI/180);
    //painter.drawLine   (xMap(paintX), yMap(paintY), xMap(x+rawLineSize*cos(tmp )), yMap(  y+rawLineSize*sin(tmp) ) );

     //draw line(direction angle of robot)
    painter.drawLine   (x, y, x+rawLineSize*cos(yaw), y+rawLineSize*sin(yaw));
    //painter.drawEllipse(xMap(paintX), yMap(paintY)-halfSize, halfSize*2,  halfSize*2);
    //draw circle(robot position)
    painter.drawEllipse(x-halfSize, y-halfSize, halfSize*2,  halfSize*2);


    //painter.drawLine   ( xMap(5), yMap(500), xMap(100),yMap(100) );

//    if(saveImg){
//       imgObj_saveIndex++;
//       char fName[128];
//       sprintf(fName,"pf_%d_%d.png",1000,imgObj_saveIndex);
//       imgObj.save(fName);
//    }

    try{
         ui->imgLB->setPixmap(QPixmap::fromImage(imgObj));
         ui->imgLB->show();
    } catch (std::exception &e) {
        //qFatal("Error %s sending event %s to object %s (%s)",
        //     e.what(), typeid(*event).name(), qPrintable(receiver->objectName()),
        //    typeid(*receiver).name());
        std::cout << "ClientWindow::updateInfo() error" << e.what() << std::endl;
    } catch (...) {
        std::cout << "ClientWindow::updateInfo() error" << std::endl;
    }

}


//void ImageWidget::paintParticle_slot(std::vector<Vector2i> pAry){
void ImageWidget::paintParticle_slot(std::vector<Vector2i> pAry,Vector2i predictPos){
    int halfSize = 2;       //機器人再圖上的半徑
    //int rawLineSize = 20;//車頭方向
     int paintX,paintY;

    particleImg = oriImg.copy();
    QPainter painter (&particleImg);

    //paint predictPos
    painter.setBrush(Qt::yellow);   //fill Circle
    paintX = predictPos(0)-halfSize*2 ;
    paintY = predictPos(1)+halfSize*2 ;
    //kbe tmp ignore
    painter.drawEllipse(paintX, paintY, halfSize*4,  halfSize*4);
    painter.setBrush(QColor(255,128,0));   //fill Circle


    for(int i=0;i < pAry.size();i++){
        paintX = pAry[i](0)-halfSize ;
        paintY = pAry[i](1)+halfSize ;

        //draw line(direction angle of robot)
        //painter.drawLine   (xMap(x), yMap(y), xMap(x+rawLineSize*cos(yaw)), yMap( y+rawLineSize*sin(yaw) ));
        //draw circle(robot position)
        //kbe tmp ignore
        painter.drawEllipse(paintX, paintY, halfSize*2,  halfSize*2);
    }


    try{
        pfCount++;
         ui->imgLB->setPixmap(QPixmap::fromImage(particleImg));

//         char fName[128];
//         sprintf(fName,"pf_%d_%d.png",1000,pfCount);
//         particleImg.save(fName);


    } catch (std::exception &e) {
        std::cout << "ClientWindow::paintParticle_slot() error" << e.what() << std::endl;
    } catch (...) {
        std::cout << "ClientWindow::paintParticle_slot() error" << std::endl;
    }



}


void ImageWidget::paintSensorLine_slot(Vector2i robot,std::vector<Vector2i> addAry){
    //particleImg = oriImg.copy();
    QPainter painter (&particleImg);
    painter.setPen(Qt::blue);


    int rx = robot(0);
    int ry = robot(1);
    //painter.setPen(Qt::blue);

    //std::cout << "robot=" << robot << std::endl;
    /*for(int i = 0;i < addAry.size();i++){
        std::cout << "addAry[" << i << "]=" << addAry[i] << std::endl;
    }*/

    for(int i= 0; i< addAry.size();i++){
        //draw line
        //painter.drawLine (rx, ry, xMap(rx+addAry[i](0) ), yMap(ry+addAry[i](1) ));
        painter.drawLine (rx, ry, xMap(rx+addAry[i](0) ), ry+addAry[i](1) );
    }


    try{
         ui->imgLB->setPixmap(QPixmap::fromImage(particleImg));
    } catch (std::exception &e) {
        std::cout << "ClientWindow::paintParticle_slot() error" << e.what() << std::endl;
    } catch (...) {
        std::cout << "ClientWindow::paintParticle_slot() error" << std::endl;
    }


}

void ImageWidget::paintImu_slot(Vector2i robot,Vector2i prerobot)
{
    QPainter painter(&particleImg);
    painter.setPen(Qt::green);
    int rx = robot(0);
    int ry = robot(1);
    painter.drawLine(prerobot(0),prerobot(1),rx,ry);
}
