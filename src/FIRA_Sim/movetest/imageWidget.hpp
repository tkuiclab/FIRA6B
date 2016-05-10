#ifndef WIDGET_H
#define WIDGET_H

#include"qpainter.h"



#ifndef Q_MOC_RUN
#include <QWidget>
#include "common/ui_image_widget.h"
//#include "qnode.hpp"
#include "../movetest/qClient.hpp"
 #include <QMetaType>
#include <QMouseEvent>
#endif

#include <eigen3/Eigen/Dense>

//#define PI 3.14
//#define def_rad2deg 180/PI
using namespace Eigen;

static int pfCount = 0;

enum PaintMode{SeriesPaint = 1,PointPaint};
class ImageWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit ImageWidget(QWidget *parent = 0);
    ~ImageWidget();

    static const int resolution = 1;  //1 meter(grid) 10 pixel

    void setImgObj(QImage inImageObj){
        oriImg = inImageObj.copy();
        imgObj= inImageObj.copy();
        particleImg = inImageObj.copy();

         ui->imgLB->setPixmap(QPixmap::fromImage(imgObj));
    }

    void setPixmap(QPixmap inPixmap){
        ui->imgLB->setPixmap(inPixmap);
    }


    PaintMode mPaintMode;
    void setPaintMode(PaintMode inMode){
        mPaintMode = inMode;
    }

    double xMap(double inX){
        if( inX <0)   return 0;
        else          return inX;
    }


    double yMap(double inY){
        double mapNum =imgObj.size().height() -inY;
        if( mapNum <0)   return 0;
        else             return mapNum;
    }


public Q_SLOTS:
    void paintRobot_slot(double x,double y,double yaw,bool saveImg);
    void paintParticle_slot(std::vector<Vector2i> pAry,Vector2i predictPos);
    void paintSensorLine_slot(Vector2i robot,std::vector<Vector2i> addAry);
    void paintImu_slot(Vector2i,Vector2i);



private:
    Ui::ImageWidget *ui;
    //QClient mClient;

     QPainter *painter;
    QImage imgObj;
    int imgObj_saveIndex;

    QImage particleImg;
    QImage oriImg;


    //happen on click
    //void mouseMoveEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *mouseEvent){
        //qDebug() << event->pos();
        //ui->ImageWidget.
        //char tStr[255];
        //sprintf(tStr,"(%d,%d)",event->x(),event->y());
        double imgX = xMap( mouseEvent->pos().x())/10.0;
        double imgY = yMap( mouseEvent->pos().y()- 75.0)/10.0;
        //int imgX = ( mouseEvent->pos().x()- ui->imgLB->x());
        //int imgY = ( mouseEvent->pos().y()- ui->imgLB->y());

        setWindowTitle(QString("Mouse move (%1,%2)").arg(imgX).arg(imgY));
        //qDebug() << QString("Mouse move (%1,%2)").arg(mouseEvent->pos().x()).arg(mouseEvent->pos().y());
    }
};

#endif // WIDGET_H
