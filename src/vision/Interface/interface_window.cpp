#include "interface_window.hpp"
#include "math.h"

#define PI 3.14159265

#define REDITEM 0x01
#define GREENITEM 0x02
#define BLUEITEM 0x04
#define YELLOWITEM 0x08

#define FILE_PATH "/tmp/HSVcolormap.bin"
#define IMAGE_TEST1 "src/vision/1.bmp"
using namespace std;
using namespace cv;

interface_window::interface_window(QInterface *node, QWidget *parent) :
    QMainWindow(parent),
    interface(node),
    ui(new Ui::interface_window)
{
    ui->setupUi(this);
    interface->on_init();
    startTimer(10);
/////////////////////////////////中心點前置參數////////////////////////////
    interface->get_center();
    if(interface->center_get_x != 0){
        ui->Slider_X->setValue(interface->center_get_x);
        ui->Slider_Y->setValue(interface->center_get_y);
        ui->Slider_Inner->setValue(interface->center_get_inner);
        ui->Slider_Outer->setValue(interface->center_get_outer);
        ui->Slider_Front->setValue(interface->center_get_front);
    }
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////掃瞄點前置參數////////////////////////////
    interface->get_scan();
    if(!interface->scan_get_para.empty()){
        ui->Scane_NEARGAP->setValue(interface->scan_get_para[0]);
        ui->Scane_NEARANGLE->setValue(interface->scan_get_para[1]);
        ui->Scane_NEARPIXEL->setValue(interface->scan_get_para[2]);
        ui->Scane_MIDDLEGAP->setValue(interface->scan_get_para[3]);
        ui->Scane_MIDDLEPIXEL->setValue(interface->scan_get_para[4]);
        ui->Scane_FARGAP->setValue(interface->scan_get_para[5]);
        ui->Scane_FARPIXEL->setValue(interface->scan_get_para[6]);
        ui->Scane_ENDGAP->setValue(interface->scan_get_para[7]);
    }
//////////////////////////////////////////////////////////////////////////
///////////////////////////////距離前置參數/////////////////////////////////
    ui->Dis_comboBox->addItem(" File ");
    ui->Dis_space_value->setNum(ui->Dis_space->value());
    ui->Dis_num_value->setNum(ui->Dis_num->value());
    for(int i=0;i<100;i++) distance_pixel[i] = 0;
    for(int i=0;i<ui->Dis_num->value();i++){
        distance_space[i]=i*ui->Dis_space->value();
    }
    distance_pixel[0]=0;   distance_pixel[1]=21;   distance_pixel[2]=46;
    distance_pixel[3]=66;  distance_pixel[4]=82;   distance_pixel[5]=94;
    distance_pixel[6]=102; distance_pixel[7]=109;  distance_pixel[8]=115;
    distance_pixel[9]=121; distance_pixel[10]=125; distance_pixel[11]=129;
    for(int i=0;i<ui->Dis_num->value();i++){
        ui->Dis_comboBox->addItem("Distance : " + QString::number(distance_space[i]));
    }
    for(int i=0;i<ui->Dis_num->value();i++){
        ui->Dis_listWidget->addItem("Distance : " + QString::number(distance_space[i])
                                    + " cm -> " + QString::number(distance_pixel[i]));
    }
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////HSV前置參數//////////////////////////////
    HSV_init[0] = 360; HSV_init[1] = 359;
    HSV_init[2] = 255; HSV_init[3] = 0;
    HSV_init[4] = 255; HSV_init[5] = 0;
    interface->get_hsv();
    if(!interface->Redmap.empty()){
        HSV_red = interface->Redmap;   HSV_green = interface->Greenmap;
        HSV_blue = interface->Bluemap; HSV_yellow = interface->Yellowmap;
        ui->HSV_Huemax->setValue(HSV_red[0]);          ui->HSV_Huemin->setValue(HSV_red[1]);
        ui->HSV_SaturationMax->setValue(HSV_red[2]);   ui->HSV_SaturationMin->setValue(HSV_red[3]);
        ui->HSV_BrightnessMax->setValue(HSV_red[4]);   ui->HSV_BrightnessMin->setValue(HSV_red[5]);
    }else{
        for(int i=0;i<6;i++){
            HSV_red.push_back(HSV_init[i]);  HSV_green.push_back(HSV_init[i]);
            HSV_blue.push_back(HSV_init[i]); HSV_yellow.push_back(HSV_init[i]);
        }
    }
/////////////////////////////////////////////////////////////////////////
}

interface_window::~interface_window()
{
    delete ui;
}
void interface_window::timerEvent(QTimerEvent *)
{
    int center_x = ui->Slider_X->value();  int center_y = ui->Slider_Y->value();
    int inner = ui->Slider_Inner->value(); int outer = ui->Slider_Outer->value();
    int front = ui->Slider_Front->value();
    if(ros::ok()){
        if(interface->cv_ptr != NULL){
            Mat frame(Size(interface->cv_ptr->image.cols,interface->cv_ptr->image.rows),CV_8UC3);
            for(int i=0;i<frame.rows*frame.cols*3;i++)frame.data[i] = interface->cv_ptr->image.data[i];
            opposite(frame);
            //frame = interface->cv_ptr->image;
            //frame = imread( IMAGE_TEST1 , CV_LOAD_IMAGE_COLOR );

            double frame_HSV[frame.rows*frame.cols*3];
            if(ui->tabModel->currentIndex()==0){
                ui->Exposure_num->setText(QString("%1").arg(ui->Slider_Exposure->value()));
                ui->White_num_R->setText(QString("%1").arg(ui->Slider_White_R->value()));
                ui->White_num_B->setText(QString("%1").arg(ui->Slider_White_B->value()));
            }
            else if(ui->tabModel->currentIndex()==1){//中心點
                Center(frame, center_x, center_y, inner, outer, front);
                if(ui->Center_sent->isDown()==true){
                    interface->sent_center(center_x, center_y, inner, outer, front);
                    ui->Center_sent->setDown(0);
                }
            }
            else if(ui->tabModel->currentIndex()==2){//掃描參數
                Scan(frame, center_x, center_y);
            }
            else if(ui->tabModel->currentIndex()==3){
               Distance_draw(frame, center_x, center_y);
            }
            else if(ui->tabModel->currentIndex()==4){
                if(ui->tabColor->currentIndex()==0){
                    RGBtoHSV(frame, frame_HSV);
                    HSVModel(frame, frame_HSV);
                    HSV_PrintAngle();
                    HSV_PrintBackground();
                    if(ui->HSV_makefile->isDown()==true){
                        HSVmap();
                        ui->HSV_makefile->setDown(0);
                    }
                }
                else if(ui->tabColor->currentIndex()==1){
                    White_Line(frame,front,center_x,center_y,inner,outer);
                    Draw_inner_outer_circle(frame,center_x,center_y,inner,outer);
                    Draw_Front_Line(frame,center_x,center_y,inner);
                }
                else if(ui->tabColor->currentIndex()==2){
                    Black_Line(frame,front,center_x,center_y,inner,outer);
                    Draw_inner_outer_circle(frame,center_x,center_y,inner,outer);
                    Draw_Front_Line(frame,center_x,center_y,inner);
                }
            }
            else if(ui->tabModel->currentIndex()==5){
                Data_check(frame,center_x,center_y);
            }
            Showimg(frame);
        }
    }
}
///////////////////////////////鏡像矯正////////////////////////////////////
void interface_window::opposite(Mat frame){
    Mat Outing(Size(frame.cols,frame.rows),CV_8UC3);
    for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
            Outing.data[(i*Outing.cols*3)+(j*3)+0] = frame.data[(i*frame.cols*3)+((frame.cols-j-1)*3)+0];
            Outing.data[(i*Outing.cols*3)+(j*3)+1] = frame.data[(i*frame.cols*3)+((frame.cols-j-1)*3)+1];
            Outing.data[(i*Outing.cols*3)+(j*3)+2] = frame.data[(i*frame.cols*3)+((frame.cols-j-1)*3)+2];
        }
    }
    for(int i=0;i<frame.rows*frame.cols*3;i++)frame.data[i] = Outing.data[i];
}
/////////////////////////////////////////////////////////////////////////
///////////////////////////////影像輸出////////////////////////////////////
void interface_window::Showimg(Mat frame){

    if(ui->check_cam->isChecked()){
        QImage img(frame.cols,frame.rows,QImage::Format_RGB888);
        for(int i=0;i<frame.rows;i++){
            for(int j=0;j<frame.cols;j++){
                unsigned char B = frame.data[(i*frame.cols*3)+(j*3)+0];
                unsigned char G = frame.data[(i*frame.cols*3)+(j*3)+1];
                unsigned char R = frame.data[(i*frame.cols*3)+(j*3)+2];
                img.setPixel(j,i,QColor(R,G,B).rgb());
            }
        }
        ui->Show_label->setPixmap(QPixmap::fromImage(img));
    }else{
        ui->Show_label->clear();
    }
}
/////////////////////////////////////////////////////////////////////////
///////////////////////////////攝影機/////////////////////////////////////
void interface_window::on_Slider_Exposure_valueChanged(int value)
{
    interface->set_campara(ui->Slider_Exposure->value(),
                           ui->Slider_White_R->value(),
                           ui->Slider_White_B->value());
}
void interface_window::on_Slider_White_R_valueChanged(int value)
{
    interface->set_campara(ui->Slider_Exposure->value(),
                           ui->Slider_White_R->value(),
                           ui->Slider_White_B->value());
}
void interface_window::on_Slider_White_B_valueChanged(int value)
{
    interface->set_campara(ui->Slider_Exposure->value(),
                           ui->Slider_White_R->value(),
                           ui->Slider_White_B->value());
}
/////////////////////////////////////////////////////////////////////////
///////////////////////////////中心點/////////////////////////////////////
void interface_window::Center(Mat frame, int center_x, int center_y,
                             int inner, int outer, int front)
{
    for(int i=center_y-1;i<center_y+1;i++){
        for(int j=center_x-30;j<center_x+30;j++){
            frame.data[(i*frame.cols*3)+(j*3)+0] = 0;
            frame.data[(i*frame.cols*3)+(j*3)+1] = 255;
            frame.data[(i*frame.cols*3)+(j*3)+2] = 0;
        }
    }
    for(int i=center_y-30;i<center_y+30;i++){
        for(int j=center_x-1;j<center_x+1;j++){
            frame.data[(i*frame.cols*3)+(j*3)+0] = 0;
            frame.data[(i*frame.cols*3)+(j*3)+1] = 255;
            frame.data[(i*frame.cols*3)+(j*3)+2] = 0;
        }
    }
    double angle = front*PI/180;
    double x = cos(angle);
    double y = sin(angle);
    for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
            int dis = sqrt(pow(i-center_y,2)+pow(j-center_x,2));
            if((dis==inner)||(dis==outer)){
                frame.data[(i*frame.cols*3)+(j*3)+0] = 0;
                frame.data[(i*frame.cols*3)+(j*3)+1] = 255;
                frame.data[(i*frame.cols*3)+(j*3)+2] = 0;
            }
        }
    }
    for(int r=0;r<inner;r++){
        int dis_x = x*r;
        int dis_y = y*r;
        frame.data[((center_y-dis_y)*frame.cols*3)+((center_x+dis_x)*3)+0] = 0;
        frame.data[((center_y-dis_y)*frame.cols*3)+((center_x+dis_x)*3)+1] = 255;
        frame.data[((center_y-dis_y)*frame.cols*3)+((center_x+dis_x)*3)+2] = 255;
    }
    ui->Center_X_num->setText(QString("%1").arg(ui->Slider_X->value()));
    ui->Center_Y_num->setText(QString("%1").arg(ui->Slider_Y->value()));
    ui->Inner_num->setText(QString("%1").arg(ui->Slider_Inner->value()));
    ui->Outer_num->setText(QString("%1").arg(ui->Slider_Outer->value()));
    ui->Front_num->setText(QString("%1").arg(ui->Slider_Front->value()));
}
////////////////////////////////////////////////////////////////////////
///////////////////////////////掃描參數///////////////////////////////////
void interface_window::Scan(Mat frame, int center_x, int center_y){
    for(int i=ui->Scane_NEARANGLE->value()/10;i<=360;i=i+(ui->Scane_NEARANGLE->value()/10)){
        double angle=i*M_PI/180;
        int num=((ui->Scane_MIDDLEGAP->value())-(ui->Scane_NEARGAP->value()))/(ui->Scane_NEARPIXEL->value());
        for(int j=1;j<=num;j++){
            int xx=j*ui->Scane_NEARPIXEL->value()*cos(angle);
            int yy=j*ui->Scane_NEARPIXEL->value()*sin(angle);
            int space_x=ui->Scane_NEARGAP->value()*cos(angle);
            int space_y=ui->Scane_NEARGAP->value()*sin(angle);
            frame.data[((center_y-yy-space_y)*frame.cols*3)+((center_x+xx+space_x)*3)+0]=0;
            frame.data[((center_y-yy-space_y)*frame.cols*3)+((center_x+xx+space_x)*3)+1]=255;
            frame.data[((center_y-yy-space_y)*frame.cols*3)+((center_x+xx+space_x)*3)+2]=0;
            if(ui->Scane_Sent->isDown()==true){
                scan_near.push_back(center_x+xx+space_x);
                scan_near.push_back(center_y-yy-space_y);
            }
        }
    }
    for(int i=ui->Scane_NEARANGLE->value()/10;i<=360;i=i+(ui->Scane_NEARANGLE->value()/10)){
        double angle=i*M_PI/180;
        int num=((ui->Scane_FARGAP->value())-(ui->Scane_MIDDLEGAP->value()))/(ui->Scane_MIDDLEPIXEL->value());
        for(int j=1;j<=num;j++){
            int xx=j*ui->Scane_MIDDLEPIXEL->value()*cos(angle);
            int yy=j*ui->Scane_MIDDLEPIXEL->value()*sin(angle);
            int space_x=ui->Scane_MIDDLEGAP->value()*cos(angle);
            int space_y=ui->Scane_MIDDLEGAP->value()*sin(angle);
            frame.data[((center_y-yy-space_y)*frame.cols*3)+((center_x+xx+space_x)*3)+0]=0;
            frame.data[((center_y-yy-space_y)*frame.cols*3)+((center_x+xx+space_x)*3)+1]=255;
            frame.data[((center_y-yy-space_y)*frame.cols*3)+((center_x+xx+space_x)*3)+2]=0;
            if(ui->Scane_Sent->isDown()==true){
                scan_middle.push_back(center_x+xx+space_x);
                scan_middle.push_back(center_y-yy-space_y);
            }
        }
        for(int j=1;j<=num;j++){
            double angle2 = angle+(ui->Scane_NEARANGLE->value()/20*M_PI/180);
            int xx2=j*ui->Scane_MIDDLEPIXEL->value()*cos(angle2);
            int yy2=j*ui->Scane_MIDDLEPIXEL->value()*sin(angle2);
            int space_x2=ui->Scane_MIDDLEGAP->value()*cos(angle2);
            int space_y2=ui->Scane_MIDDLEGAP->value()*sin(angle2);
            frame.data[((center_y-yy2-space_y2)*frame.cols*3)+((center_x+xx2+space_x2)*3)+0]=0;
            frame.data[((center_y-yy2-space_y2)*frame.cols*3)+((center_x+xx2+space_x2)*3)+1]=255;
            frame.data[((center_y-yy2-space_y2)*frame.cols*3)+((center_x+xx2+space_x2)*3)+2]=0;
            if(ui->Scane_Sent->isDown()==true){
                scan_middle.push_back(center_x+xx2+space_x2);
                scan_middle.push_back(center_y-yy2-space_y2);
            }
        }
    }
    for(int i=ui->Scane_NEARANGLE->value()/10;i<=360;i=i+(ui->Scane_NEARANGLE->value()/10)){
        double angle=i*M_PI/180;
        double angle2 = angle+(ui->Scane_NEARANGLE->value()/20*M_PI/180);
        double angle3 = angle+(ui->Scane_NEARANGLE->value()/40*M_PI/180);
        double angle4 = angle2+(ui->Scane_NEARANGLE->value()/40*M_PI/180);
        int num=((ui->Scane_ENDGAP->value())-(ui->Scane_FARGAP->value()))/(ui->Scane_FARPIXEL->value());
        for(int j=1;j<=num;j++){
            int xx=j*ui->Scane_FARPIXEL->value()*cos(angle);
            int yy=j*ui->Scane_FARPIXEL->value()*sin(angle);
            int space_x=ui->Scane_FARGAP->value()*cos(angle);
            int space_y=ui->Scane_FARGAP->value()*sin(angle);
            frame.data[((center_y-yy-space_y)*frame.cols*3)+((center_x+xx+space_x)*3)+0]=0;
            frame.data[((center_y-yy-space_y)*frame.cols*3)+((center_x+xx+space_x)*3)+1]=255;
            frame.data[((center_y-yy-space_y)*frame.cols*3)+((center_x+xx+space_x)*3)+2]=0;
            if(ui->Scane_Sent->isDown()==true){
                scan_far.push_back(center_x+xx+space_x);
                scan_far.push_back(center_y-yy-space_y);
            }
        }
        for(int j=1;j<=num;j++){
            int xx3=j*ui->Scane_FARPIXEL->value()*cos(angle3);
            int yy3=j*ui->Scane_FARPIXEL->value()*sin(angle3);
            int space_x3=ui->Scane_FARGAP->value()*cos(angle3);
            int space_y3=ui->Scane_FARGAP->value()*sin(angle3);
            frame.data[((center_y-yy3-space_y3)*frame.cols*3)+((center_x+xx3+space_x3)*3)+0]=0;
            frame.data[((center_y-yy3-space_y3)*frame.cols*3)+((center_x+xx3+space_x3)*3)+1]=255;
            frame.data[((center_y-yy3-space_y3)*frame.cols*3)+((center_x+xx3+space_x3)*3)+2]=0;
            if(ui->Scane_Sent->isDown()==true){
                scan_far.push_back(center_x+xx3+space_x3);
                scan_far.push_back(center_y-yy3-space_y3);
            }
        }
        for(int j=1;j<=num;j++){
            int xx2=j*ui->Scane_FARPIXEL->value()*cos(angle2);
            int yy2=j*ui->Scane_FARPIXEL->value()*sin(angle2);
            int space_x2=ui->Scane_FARGAP->value()*cos(angle2);
            int space_y2=ui->Scane_FARGAP->value()*sin(angle2);
            frame.data[((center_y-yy2-space_y2)*frame.cols*3)+((center_x+xx2+space_x2)*3)+0]=0;
            frame.data[((center_y-yy2-space_y2)*frame.cols*3)+((center_x+xx2+space_x2)*3)+1]=255;
            frame.data[((center_y-yy2-space_y2)*frame.cols*3)+((center_x+xx2+space_x2)*3)+2]=0;
            if(ui->Scane_Sent->isDown()==true){
                scan_far.push_back(center_x+xx2+space_x2);
                scan_far.push_back(center_y-yy2-space_y2);
            }
        }
        for(int j=1;j<=num;j++){
            int xx4=j*ui->Scane_FARPIXEL->value()*cos(angle4);
            int yy4=j*ui->Scane_FARPIXEL->value()*sin(angle4);
            int space_x4=ui->Scane_FARGAP->value()*cos(angle4);
            int space_y4=ui->Scane_FARGAP->value()*sin(angle4);
            frame.data[((center_y-yy4-space_y4)*frame.cols*3)+((center_x+xx4+space_x4)*3)+0]=0;
            frame.data[((center_y-yy4-space_y4)*frame.cols*3)+((center_x+xx4+space_x4)*3)+1]=255;
            frame.data[((center_y-yy4-space_y4)*frame.cols*3)+((center_x+xx4+space_x4)*3)+2]=0;
            if(ui->Scane_Sent->isDown()==true){
                scan_far.push_back(center_x+xx4+space_x4);
                scan_far.push_back(center_y-yy4-space_y4);
            }
        }
    }
    if(ui->Scane_Sent->isDown()==true){
        scan_para.push_back(ui->Scane_NEARGAP->value());
        scan_para.push_back(ui->Scane_NEARANGLE->value());
        scan_para.push_back(ui->Scane_NEARPIXEL->value());
        scan_para.push_back(ui->Scane_MIDDLEGAP->value());
        scan_para.push_back(ui->Scane_MIDDLEPIXEL->value());
        scan_para.push_back(ui->Scane_FARGAP->value());
        scan_para.push_back(ui->Scane_FARPIXEL->value());
        scan_para.push_back(ui->Scane_ENDGAP->value());
        interface->sent_scan(scan_para, scan_near, scan_middle, scan_far);
        ui->Scane_Sent->setDown(0);
        scan_para.clear();scan_near.clear();
        scan_middle.clear();scan_far.clear();
    }
}
////////////////////////////////////////////////////////////////////////
///////////////////////////////滑鼠觸發///////////////////////////////////
void interface_window::mouseMoveEvent(QMouseEvent *event)
{
    mosue_x = event->x()-ui->frame_38->x()-ui->Slider_X->value();
    mosue_y = -1*(event->y()-15-ui->Slider_Y->value());
    ui->mouse_X->setText(QString("%1").arg(mosue_x));
    ui->mouse_Y->setText(QString("%1").arg(mosue_y));
}
void interface_window::mousePressEvent(QMouseEvent *event)
{
    mosue_x = event->x()-ui->frame_38->x()-ui->Slider_X->value();
    mosue_y = -1*(event->y()-15-ui->Slider_Y->value());
    if(ui->tabModel->currentIndex()==3){
        if(ui->Dis_comboBox->currentIndex()!=0){
            int Dis=hypot(mosue_x,mosue_y);
            distance_pixel[ui->Dis_comboBox->currentIndex()-1]=Dis;
            ui->Dis_listWidget->clear();
            for(int i=0;i<ui->Dis_num->value();i++){
                ui->Dis_listWidget->addItem("Distance : " + QString::number(distance_space[i])
                                            + " cm -> " + QString::number(distance_pixel[i]));
            }
        }
    }
    ui->mouse_X->setText(QString("%1").arg(mosue_x));
    ui->mouse_Y->setText(QString("%1").arg(mosue_y));
}
////////////////////////////////////////////////////////////////////////
///////////////////////////////距離參數///////////////////////////////////
void interface_window::on_Dis_space_valueChanged(int value)
{
    dis_combox_and_list_renew();
}
void interface_window::on_Dis_num_valueChanged(int value)
{
    dis_combox_and_list_renew();
}
void interface_window::dis_combox_and_list_renew()
{
    ui->Dis_space_value->setNum(ui->Dis_space->value());
    ui->Dis_num_value->setNum(ui->Dis_num->value());
    ui->Dis_comboBox->clear();
    ui->Dis_listWidget->clear();
    ui->Dis_comboBox->addItem(" File ");
    for(int i=0;i<ui->Dis_num->value();i++){
        distance_space[i]=i*ui->Dis_space->value();
    }
    for(int i=0;i<ui->Dis_num->value();i++){
        ui->Dis_comboBox->addItem("Distance : " + QString::number(distance_space[i]) );
    }
    for(int i=0;i<ui->Dis_num->value();i++){
        ui->Dis_listWidget->addItem("Distance : " + QString::number(distance_space[i])
                                    + " cm -> " + QString::number(distance_pixel[i]));
    }
}
 void interface_window::Distance_draw(Mat frame, int center_x, int center_y){
     for(int i=0;i<distance_pixel[ui->Dis_num->value()-1];i++){
         frame.data[(center_y*frame.cols*3)+((center_x+i)*3)+0] = 0;
         frame.data[(center_y*frame.cols*3)+((center_x+i)*3)+1] = 255;
         frame.data[(center_y*frame.cols*3)+((center_x+i)*3)+2] = 0;

         frame.data[(center_y*frame.cols*3)+((center_x-i)*3)+0] = 0;
         frame.data[(center_y*frame.cols*3)+((center_x-i)*3)+1] = 255;
         frame.data[(center_y*frame.cols*3)+((center_x-i)*3)+2] = 0;

         frame.data[((center_y+i)*frame.cols*3)+(center_x*3)+0] = 0;
         frame.data[((center_y+i)*frame.cols*3)+(center_x*3)+1] = 255;
         frame.data[((center_y+i)*frame.cols*3)+(center_x*3)+2] = 0;

         frame.data[((center_y-i)*frame.cols*3)+(center_x*3)+0] = 0;
         frame.data[((center_y-i)*frame.cols*3)+(center_x*3)+1] = 255;
         frame.data[((center_y-i)*frame.cols*3)+(center_x*3)+2] = 0;
     }
     for(int i=1;i<ui->Dis_num->value();i++){
         for(int j=-5;j<=5;j++){
             frame.data[((center_y+j)*frame.cols*3)+((center_x+distance_pixel[i])*3)+0] = 0;
             frame.data[((center_y+j)*frame.cols*3)+((center_x+distance_pixel[i])*3)+1] = 255;
             frame.data[((center_y+j)*frame.cols*3)+((center_x+distance_pixel[i])*3)+2] = 0;

             frame.data[((center_y+j)*frame.cols*3)+((center_x-distance_pixel[i])*3)+0] = 0;
             frame.data[((center_y+j)*frame.cols*3)+((center_x-distance_pixel[i])*3)+1] = 255;
             frame.data[((center_y+j)*frame.cols*3)+((center_x-distance_pixel[i])*3)+2] = 0;

             frame.data[((center_y+distance_pixel[i])*frame.cols*3)+((center_x+j)*3)+0] = 0;
             frame.data[((center_y+distance_pixel[i])*frame.cols*3)+((center_x+j)*3)+1] = 255;
             frame.data[((center_y+distance_pixel[i])*frame.cols*3)+((center_x+j)*3)+2] = 0;

             frame.data[((center_y-distance_pixel[i])*frame.cols*3)+((center_x+j)*3)+0] = 0;
             frame.data[((center_y-distance_pixel[i])*frame.cols*3)+((center_x+j)*3)+1] = 255;
             frame.data[((center_y-distance_pixel[i])*frame.cols*3)+((center_x+j)*3)+2] = 0;
         }
     }
     if(ui->dis_para->isDown() ==true){

         for(int i=0;i<ui->Dis_num->value();i++){
             dis_space.push_back(distance_space[i]);
             dis_pixel.push_back(distance_pixel[i]);
         }
        interface->sent_dis(ui->Dis_space->value(),dis_space,dis_pixel);
        ui->dis_para->setDown(0);
        dis_space.clear();dis_pixel.clear();
     }
 }
////////////////////////////////////////////////////////////////////////
///////////////////////////////RGBtoHSV///////////////////////////////////
void interface_window::RGBtoHSV(Mat frame, double *frame_HSV)
{
    double Rnew,Gnew,Bnew,HSVmax,HSVmin;
    int HSVnum =0;
    double H_sum,S_sum,V_sum;
    for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
            Bnew = frame.data[(i*frame.cols*3)+(j*3)+0]/255.0;
            Gnew = frame.data[(i*frame.cols*3)+(j*3)+1]/255.0;
            Rnew = frame.data[(i*frame.cols*3)+(j*3)+2]/255.0;
            RGBtoHSV_maxmin(Rnew, Gnew, Bnew, HSVmax, HSVmin);
            H_sum = RGBtoHSV_H(Rnew, Gnew, Bnew, HSVmax, HSVmin);
            S_sum = RGBtoHSV_S(HSVmax,HSVmin);
            V_sum = HSVmax*255.0;
            frame_HSV[HSVnum++] = H_sum;
            frame_HSV[HSVnum++] = S_sum;
            frame_HSV[HSVnum++] = V_sum;
        }
    }
}
void interface_window::RGBtoHSV_maxmin(double &Rnew, double &Gnew, double &Bnew, double &HSVmax, double &HSVmin)
{
    if(Rnew >= Gnew){
        if(Rnew >= Bnew){
            HSVmax = Rnew;
            if(Gnew >= Bnew)HSVmin = Bnew;
            else HSVmin = Gnew;
        }else{
            HSVmax = Bnew;
            HSVmin = Gnew;
        }
    }else{
        if(Gnew >= Bnew){
            HSVmax = Gnew;
            if(Rnew >= Bnew)HSVmin = Bnew;
            else HSVmin = Rnew;
        }else{
            HSVmax = Bnew;
            HSVmin = Rnew;
        }
    }
}
double interface_window::RGBtoHSV_H(double Rnew, double Gnew, double Bnew, double HSVmax, double HSVmin)
{
    double dis = HSVmax-HSVmin;
    if(dis == 0){
        return 0;
    }else if(HSVmax == Rnew){
        if(Gnew >= Bnew){
            return 60.0*((Gnew-Bnew)/dis);
        }else{
            return 60.0*((Gnew-Bnew)/dis+6);
        }
    }else if(HSVmax == Gnew){
        return 60.0*((Bnew-Rnew)/dis+2);
    }else if(HSVmax == Bnew){
        return 60.0*((Rnew-Gnew)/dis+4);
    }
}
double interface_window::RGBtoHSV_S(double HSVmax, double HSVmin)
{
    double dis = HSVmax-HSVmin;
    if(dis == 0){
        return 0;
    }else{
        return dis/HSVmax*255.0;
    }
}
////////////////////////////////////////////////////////////////////////
///////////////////////////////HSVtoRGB///////////////////////////////////
void interface_window::HSVtoRGB( double Hvalue, double Svalue, double Vvalue, int &Rvalue, int &Gvalue, int &Bvalue)
{
    int hi;
    double f,p,q,t;
    if(Hvalue==0.0 && Svalue==0.0 && Vvalue==0.0){
        Rvalue = 0;
        Gvalue = 0;
        Bvalue = 0;
    }else if(Svalue == 0.0){
        Rvalue = Vvalue*255.0;
        Gvalue = Vvalue*255.0;
        Bvalue = Vvalue*255.0;
    }else{
        hi = ((int)Hvalue/60)%6;
        f = Hvalue/60.0 - (double)hi;
        p = Vvalue*(1.0-Svalue);
        q = Vvalue*(1.0-f*Svalue);
        t = Vvalue*(1.0-(1.0-f)*Svalue);
        switch(hi){
        case 1:{
            Rvalue = q*255.0;
            Gvalue = Vvalue*255.0;
            Bvalue = p*255.0;
            break;
        }case 2:{
            Rvalue = p*255.0;
            Gvalue = Vvalue*255.0;
            Bvalue = t*255.0;
            break;
        }case 3:{
            Rvalue = p*255.0;
            Gvalue = q*255.0;
            Bvalue = Vvalue*255.0;
            break;
        }case 4:{
            Rvalue = t*255.0;
            Gvalue = p*255.0;
            Bvalue = Vvalue*255.0;
            break;
        }case 5:{
            Rvalue = Vvalue*255.0;
            Gvalue = p*255.0;
            Bvalue = q*255.0;
            break;
        }default:{
            Rvalue = Vvalue*255.0;
            Gvalue = t*255.0;
            Bvalue = p*255.0;
            break;
        }
        }
    }
}
////////////////////////////////////////////////////////////////////////
///////////////////////////////色彩搜寻///////////////////////////////////
void interface_window::on_HSV_comboBox_currentIndexChanged(int index)
{
    if(ui->HSV_comboBox->currentIndex()==0){
        ui->HSV_Huemax->setValue(HSV_red[0]);          ui->HSV_Huemin->setValue(HSV_red[1]);
        ui->HSV_SaturationMax->setValue(HSV_red[2]);   ui->HSV_SaturationMin->setValue(HSV_red[3]);
        ui->HSV_BrightnessMax->setValue(HSV_red[4]);   ui->HSV_BrightnessMin->setValue(HSV_red[5]);
    }else if(ui->HSV_comboBox->currentIndex()==1){
        ui->HSV_Huemax->setValue(HSV_green[0]);          ui->HSV_Huemin->setValue(HSV_green[1]);
        ui->HSV_SaturationMax->setValue(HSV_green[2]);   ui->HSV_SaturationMin->setValue(HSV_green[3]);
        ui->HSV_BrightnessMax->setValue(HSV_green[4]);   ui->HSV_BrightnessMin->setValue(HSV_green[5]);
    }else if(ui->HSV_comboBox->currentIndex()==2){
        ui->HSV_Huemax->setValue(HSV_blue[0]);          ui->HSV_Huemin->setValue(HSV_blue[1]);
        ui->HSV_SaturationMax->setValue(HSV_blue[2]);   ui->HSV_SaturationMin->setValue(HSV_blue[3]);
        ui->HSV_BrightnessMax->setValue(HSV_blue[4]);   ui->HSV_BrightnessMin->setValue(HSV_blue[5]);
    }else if(ui->HSV_comboBox->currentIndex()==3){
        ui->HSV_Huemax->setValue(HSV_yellow[0]);          ui->HSV_Huemin->setValue(HSV_yellow[1]);
        ui->HSV_SaturationMax->setValue(HSV_yellow[2]);   ui->HSV_SaturationMin->setValue(HSV_yellow[3]);
        ui->HSV_BrightnessMax->setValue(HSV_yellow[4]);   ui->HSV_BrightnessMin->setValue(HSV_yellow[5]);
    }
}
void interface_window::HSVModel(Mat frame, double *frame_HSV)
{
    if(ui->HSV_reset->isDown() == true){
        ui->HSV_Huemax->setValue(HSV_init[0]);          ui->HSV_Huemin->setValue(HSV_init[1]);
        ui->HSV_SaturationMax->setValue(HSV_init[2]);   ui->HSV_SaturationMin->setValue(HSV_init[3]);
        ui->HSV_BrightnessMax->setValue(HSV_init[4]);   ui->HSV_BrightnessMin->setValue(HSV_init[5]);
    }
    for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
            if(ui->HSV_Huemin->value()<ui->HSV_Huemax->value()){
                if(  (frame_HSV[(i*frame.cols*3)+(j*3)+0] >= ui->HSV_Huemin->value())
                    &&(frame_HSV[(i*frame.cols*3)+(j*3)+0] <= ui->HSV_Huemax->value())
                    &&(frame_HSV[(i*frame.cols*3)+(j*3)+1] >= ui->HSV_SaturationMin->value())
                    &&(frame_HSV[(i*frame.cols*3)+(j*3)+1] <= ui->HSV_SaturationMax->value())
                    &&(frame_HSV[(i*frame.cols*3)+(j*3)+2] >= ui->HSV_BrightnessMin->value())
                    &&(frame_HSV[(i*frame.cols*3)+(j*3)+2] <= ui->HSV_BrightnessMax->value()) )
                {
                    if(ui->HSV_comboBox->currentIndex()==0){
                        frame.data[(i*frame.cols*3)+(j*3)+0] = 255;
                        frame.data[(i*frame.cols*3)+(j*3)+1] = 0;
                        frame.data[(i*frame.cols*3)+(j*3)+2] = 255;
                    }else if(ui->HSV_comboBox->currentIndex()==1){
                        frame.data[(i*frame.cols*3)+(j*3)+0] = 0;
                        frame.data[(i*frame.cols*3)+(j*3)+1] = 0;
                        frame.data[(i*frame.cols*3)+(j*3)+2] = 255;
                    }else if(ui->HSV_comboBox->currentIndex()==2){
                        frame.data[(i*frame.cols*3)+(j*3)+0] = 0;
                        frame.data[(i*frame.cols*3)+(j*3)+1] = 255;
                        frame.data[(i*frame.cols*3)+(j*3)+2] = 0;
                    }else if(ui->HSV_comboBox->currentIndex()==3){
                        frame.data[(i*frame.cols*3)+(j*3)+0] = 255;
                        frame.data[(i*frame.cols*3)+(j*3)+1] = 0;
                        frame.data[(i*frame.cols*3)+(j*3)+2] = 0;
                    }
                }
            }else{
                if(  (frame_HSV[(i*frame.cols*3)+(j*3)+0] >= ui->HSV_Huemin->value())
                   ||(frame_HSV[(i*frame.cols*3)+(j*3)+0] <= ui->HSV_Huemax->value())
                   &&(frame_HSV[(i*frame.cols*3)+(j*3)+1] >= ui->HSV_SaturationMin->value())
                   &&(frame_HSV[(i*frame.cols*3)+(j*3)+1] <= ui->HSV_SaturationMax->value())
                   &&(frame_HSV[(i*frame.cols*3)+(j*3)+2] >= ui->HSV_BrightnessMin->value())
                   &&(frame_HSV[(i*frame.cols*3)+(j*3)+2] <= ui->HSV_BrightnessMax->value()) )
                {
                    if(ui->HSV_comboBox->currentIndex()==0){
                        frame.data[(i*frame.cols*3)+(j*3)+0] = 255;
                        frame.data[(i*frame.cols*3)+(j*3)+1] = 0;
                        frame.data[(i*frame.cols*3)+(j*3)+2] = 255;
                    }else if(ui->HSV_comboBox->currentIndex()==1){
                        frame.data[(i*frame.cols*3)+(j*3)+0] = 0;
                        frame.data[(i*frame.cols*3)+(j*3)+1] = 0;
                        frame.data[(i*frame.cols*3)+(j*3)+2] = 255;
                    }else if(ui->HSV_comboBox->currentIndex()==2){
                        frame.data[(i*frame.cols*3)+(j*3)+0] = 0;
                        frame.data[(i*frame.cols*3)+(j*3)+1] = 255;
                        frame.data[(i*frame.cols*3)+(j*3)+2] = 0;
                    }else if(ui->HSV_comboBox->currentIndex()==3){
                        frame.data[(i*frame.cols*3)+(j*3)+0] = 255;
                        frame.data[(i*frame.cols*3)+(j*3)+1] = 0;
                        frame.data[(i*frame.cols*3)+(j*3)+2] = 0;
                    }
                }
            }
        }
    }
    if(ui->HSV_save->isDown() == true){
        if(ui->HSV_comboBox->currentIndex()==0){
            HSV_red[0] = ui->HSV_Huemax->value();          HSV_red[1] = ui->HSV_Huemin->value();
            HSV_red[2] = ui->HSV_SaturationMax->value();   HSV_red[3] = ui->HSV_SaturationMin->value();
            HSV_red[4] = ui->HSV_BrightnessMax->value();   HSV_red[5] = ui->HSV_BrightnessMin->value();
        }else if(ui->HSV_comboBox->currentIndex()==1){
            HSV_green[0] = ui->HSV_Huemax->value();        HSV_green[1] = ui->HSV_Huemin->value();
            HSV_green[2] = ui->HSV_SaturationMax->value(); HSV_green[3] = ui->HSV_SaturationMin->value();
            HSV_green[4] = ui->HSV_BrightnessMax->value(); HSV_green[5] = ui->HSV_BrightnessMin->value();
        }else if(ui->HSV_comboBox->currentIndex()==2){
            HSV_blue[0] = ui->HSV_Huemax->value();         HSV_blue[1] = ui->HSV_Huemin->value();
            HSV_blue[2] = ui->HSV_SaturationMax->value();  HSV_blue[3] = ui->HSV_SaturationMin->value();
            HSV_blue[4] = ui->HSV_BrightnessMax->value();  HSV_blue[5] = ui->HSV_BrightnessMin->value();
        }else if(ui->HSV_comboBox->currentIndex()==3){
            HSV_yellow[0] = ui->HSV_Huemax->value();       HSV_yellow[1] = ui->HSV_Huemin->value();
            HSV_yellow[2] = ui->HSV_SaturationMax->value();HSV_yellow[3] = ui->HSV_SaturationMin->value();
            HSV_yellow[4] = ui->HSV_BrightnessMax->value();HSV_yellow[5] = ui->HSV_BrightnessMin->value();
        }
    }
}
////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////HSVmap////////////////////////////////
void interface_window::HSVmap()
{
    unsigned char *HSVmap = new unsigned char[256*256*256];
    for(int b=0;b<256;b++){
        for(int g=0;g<256;g++){
            for(int r=0;r<256;r++){
                double Rnew,Gnew,Bnew,HSVmax,HSVmin,H_sum,S_sum,V_sum;
                Bnew = b/255.0;
                Gnew = g/255.0;
                Rnew = r/255.0;
                RGBtoHSV_maxmin(Rnew, Gnew, Bnew, HSVmax, HSVmin);
                H_sum = RGBtoHSV_H(Rnew, Gnew, Bnew, HSVmax, HSVmin);
                S_sum = RGBtoHSV_S(HSVmax,HSVmin);
                V_sum = HSVmax*255.0;
                HSVmap[r+(g<<8)+(b<<16)] = 0x00;
                if(HSV_red[1] < HSV_red[0]){
                    if( (H_sum >= HSV_red[1]) && (H_sum <= HSV_red[0])
                      &&(S_sum >= HSV_red[3]) && (S_sum <= HSV_red[2])
                      &&(V_sum >= HSV_red[5]) && (V_sum <= HSV_red[4]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | REDITEM;

                }else{
                    if( (H_sum >= HSV_red[1]) || (H_sum <= HSV_red[0])
                      &&(S_sum >= HSV_red[3]) && (S_sum <= HSV_red[2])
                      &&(V_sum >= HSV_red[5]) && (V_sum <= HSV_red[4]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | REDITEM;
               }
                if(HSV_green[1] < HSV_green[0]){
                    if( (H_sum >= HSV_green[1]) && (H_sum <= HSV_green[0])
                      &&(S_sum >= HSV_green[3]) && (S_sum <= HSV_green[2])
                      &&(V_sum >= HSV_green[5]) && (V_sum <= HSV_green[4]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | GREENITEM;
                }else{
                    if( (H_sum >= HSV_green[1]) || (H_sum <= HSV_green[0])
                      &&(S_sum >= HSV_green[3]) && (S_sum <= HSV_green[2])
                      &&(V_sum >= HSV_green[5]) && (V_sum <= HSV_green[4]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | GREENITEM;
               }
                if(HSV_blue[1] < HSV_blue[0]){
                    if( (H_sum >= HSV_blue[1]) && (H_sum <= HSV_blue[0])
                      &&(S_sum >= HSV_blue[3]) && (S_sum <= HSV_blue[2])
                      &&(V_sum >= HSV_blue[5]) && (V_sum <= HSV_blue[4]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | BLUEITEM;
                }else{
                    if( (H_sum >= HSV_blue[1]) || (H_sum <= HSV_blue[0])
                      &&(S_sum >= HSV_blue[3]) && (S_sum <= HSV_blue[2])
                      &&(V_sum >= HSV_blue[5]) && (V_sum <= HSV_blue[4]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | BLUEITEM;
               }
                if(HSV_yellow[1] < HSV_yellow[0]){
                    if( (H_sum >= HSV_yellow[1]) && (H_sum <= HSV_yellow[0])
                      &&(S_sum >= HSV_yellow[3]) && (S_sum <= HSV_yellow[2])
                      &&(V_sum >= HSV_yellow[5]) && (V_sum <= HSV_yellow[4]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | YELLOWITEM;
                }else{
                    if( (H_sum >= HSV_yellow[1]) || (H_sum <= HSV_yellow[0])
                      &&(S_sum >= HSV_yellow[3]) && (S_sum <= HSV_yellow[2])
                      &&(V_sum >= HSV_yellow[5]) && (V_sum <= HSV_yellow[4]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | YELLOWITEM;
               }
            }
        }
    }
    interface->sent_hsv(HSV_red,HSV_green,HSV_blue,HSV_yellow);
    FILE *file=fopen(FILE_PATH,"wb"); //開啟檔案來寫
    fwrite( HSVmap, 1, 256*256*256 , file );
    fclose(file);
}
/// ////////////////////////////////////////////////////////////////////
////////////////////////////HSV_PrintAngle//////////////////////////////
uchar * interface_window::HSV_PrintAngle()
{
    int w = 64;
    int h = 128;
    unsigned char *arr = new unsigned char[w*h*3];
    int line ;
    int rvalue,gvalue,bvalue;
    float Angle,Mgn,Brightness;
    float tmpAngle;
    float anglestarpoint = ui->HSV_Huemax->value() / 360.0;
    float angleendpoint = ui->HSV_Huemin->value() / 360.0;
    if(angleendpoint < anglestarpoint)
    {
        if(anglestarpoint < 0.0)anglestarpoint += (2.0 * M_PI);
        if(angleendpoint < 0.0)angleendpoint += (2.0 * M_PI);
    }
    tmpAngle = (angleendpoint - anglestarpoint) / 2.0 + anglestarpoint;
    if(tmpAngle > M_PI)tmpAngle -= (2.0 * M_PI);
    tmpAngle *=(float)2.0 * (float)M_PI * 180 / M_PI;
    for(int j=0;j<h;j++){
        for(int i=0;i<w;i++){
            line = ((j*w*3)+(i*3));
            if((2*i+j)<h){
                arr[line]=255;
                arr[line+1]=255;
                arr[line+2]=255;
            }else{
                Mgn = (float)i / (float)63.0;
                Brightness = (float)j / (float)127.0;
                if(Mgn >=(float) ui->HSV_SaturationMin->value() / 255.0
                        && Mgn <= (float) ui->HSV_SaturationMax->value() / 255.0
                        &&Brightness >= (float) ui->HSV_BrightnessMin->value() /255.0
                        &&Brightness <= (float) ui->HSV_BrightnessMax->value() /255.0){
                    Angle = tmpAngle;
                }else{
                    if(tmpAngle > 180.0){
                        Angle = tmpAngle - (float)180.0;
                    }else{
                        Angle = tmpAngle + (float)180.0;
                    }
                }
                HSVtoRGB( Angle, Mgn, Brightness, rvalue, gvalue, bvalue);
                arr[line]=rvalue;
                arr[line+1]=gvalue;
                arr[line+2]=bvalue;
            }
        }
    }
    QImage img(w,h,QImage::Format_RGB888);
    QRgb val;
    for(int j=0;j<h;j++){
        for(int i=0;i<w;i++){
            line = ((j*w*3)+(i*3));
            val = qRgb(arr[line],arr[line+1],arr[line+2]);
            img.setPixel(i,j,val);
        }
    }
    ui->showlabel->setPixmap(QPixmap::fromImage(img));
    ui->showlabel->show();
    return 0;
}
////////////////////////////////////////////////////////////////////////
//////////////////////////HSV_PrintBackground///////////////////////////
uchar * interface_window::HSV_PrintBackground()
{
    int w = 128;
    int h = 128;
    unsigned char *arr = new unsigned char[w*h*3];
    int line ;
    int tmpvalue;
    int rvalue,gvalue,bvalue;
    float Angle,Mgn,Brightness;
    tmpvalue = 128 / 2 - 1;
    for(int j=0;j<h;j++){
        for(int i=0;i<w;i++){
            line = ((j*w*3)+(i*3));
            Mgn = (float) hypot((double)(j - tmpvalue), (double)(i - tmpvalue));
            Mgn /= (float)(tmpvalue);
            if(Mgn > 1.0){
                arr[line] = 255;
                arr[line+1]=255;
                arr[line+2]=255;
            }else{
                if(i - tmpvalue==0){
                    if((j - tmpvalue)<0)Angle = 0.25;
                    else Angle = 0.75;
                }else{
                    Angle = atan2((float)(j - tmpvalue), (float)(i - tmpvalue));
                    Angle = Angle / (float)M_PI *(float)0.5 + (float)0.5;
                    if(Angle >= 1.0) Angle = 0.0;
                }if(ui->HSV_SaturationMax->value() / (float)255.0 >= Mgn && ui->HSV_SaturationMin->value() / (float)255.0 <= Mgn)
                {
                    if(ui->HSV_Huemax->value() / (float)360.0 >= ui->HSV_Huemin->value() / (float)360.0
                            && ui->HSV_Huemax->value() / (float)360.0 >= Angle

                            && ui->HSV_Huemin->value() / (float)360.0 <= Angle)
                    {
                        Brightness=(float)0.9;
                    }else if(ui->HSV_Huemax->value() / (float)360.0 < ui->HSV_Huemin->value() / (float)360.0
                              && (ui->HSV_Huemax->value() / (float)360.0 >= Angle

                              || ui->HSV_Huemin->value() / (float)360.0 <= Angle))
                    {
                        Brightness=(float)0.9;
                    }else{
                        Brightness=(float)0.1;
                    }
                }
                else Brightness = (float)0.1;
                Angle = Angle * (float)360.0;
                HSVtoRGB( Angle, Mgn, Brightness, rvalue, gvalue, bvalue);
                arr[line]=rvalue;
                arr[line+1]=gvalue;
                arr[line+2]=bvalue;
            }

        }
    }
    QImage img(w,h,QImage::Format_RGB888);
    QRgb val;
    for(int j=0;j<h;j++){
        for(int i=0;i<w;i++){
            line = ((j*w*3)+(i*3));
            val = qRgb(arr[line],arr[line+1],arr[line+2]);
            img.setPixel(i,j,val);
        }
    }
    ui->showlabel2->setPixmap(QPixmap::fromImage(img));
    ui->showlabel2->show();
    return 0;
}
////////////////////////////////////////////////////////////////////////
///////////////////////////////White_Line///////////////////////////////
void interface_window::White_Line(cv::Mat frame, int front,int center_X, int center_Y,int inner, int outer)
{
    for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
            unsigned char gray = ( frame.data[(i*frame.cols*3)+(j*3)+0]
                                 + frame.data[(i*frame.cols*3)+(j*3)+1]
                                 + frame.data[(i*frame.cols*3)+(j*3)+2])/3;
            if(gray< ui->Slider_Gray->value() ){
                frame.data[(i*frame.cols*3)+(j*3)+0] = 0;
                frame.data[(i*frame.cols*3)+(j*3)+1] = 0;
                frame.data[(i*frame.cols*3)+(j*3)+2] = 0;
            }else{
                frame.data[(i*frame.cols*3)+(j*3)+0] = 255;
                frame.data[(i*frame.cols*3)+(j*3)+1] = 255;
                frame.data[(i*frame.cols*3)+(j*3)+2] = 255;
            }
        }
    }
    for(int a=0;a<360;a=a+ui->Slider_Angle->value()){
        int angle_be = a+front;
        if(angle_be>360)angle_be-=360;
        double angle_af = angle_be*M_PI/180;
        double x = cos(angle_af);
        double y = sin(angle_af);
        for(int r=inner;r<=outer;r++){
            int dis_x = x*r;
            int dis_y = y*r;
            if( frame.data[((center_Y-dis_y)*frame.cols*3)+((center_X+dis_x)*3)+0] == 255
              &&frame.data[((center_Y-dis_y)*frame.cols*3)+((center_X+dis_x)*3)+1] == 255
              &&frame.data[((center_Y-dis_y)*frame.cols*3)+((center_X+dis_x)*3)+2] == 255){
                break;
            }else{
                frame.data[((center_Y-dis_y)*frame.cols*3)+((center_X+dis_x)*3)+0] = 0;
                frame.data[((center_Y-dis_y)*frame.cols*3)+((center_X+dis_x)*3)+1] = 0;
                frame.data[((center_Y-dis_y)*frame.cols*3)+((center_X+dis_x)*3)+2] = 255;
            }
        }
    }
    if(ui->White_sent->isDown()==true){
        interface->sent_whiteline(ui->Slider_Gray->value(),ui->Slider_Angle->value());
        ui->White_sent->setDown(0);
    }
}
////////////////////////////////////////////////////////////////////////
///////////////////////////////BlackItem///////////////////////////////
void interface_window :: Black_Line(cv::Mat frame, int front,int center_X, int center_Y,int inner, int outer)
{
    Mat Outimg(cv::Size(frame.cols,frame.rows),CV_8UC3);
    double gray_num[256] = {0};
    int avg_new=0;
    int gray_sum = 0;
    int gray_avg = 0;
    int gray_low = 0;
    int gray_hight = 0;
    int avg_old = 0;
    for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
            unsigned char gray =(frame.data[(i*frame.cols*3)+(j*3)+0]+
                                frame.data[(i*frame.cols*3)+(j*3)+1]+
                                frame.data[(i*frame.cols*3)+(j*3)+2])/3;
            Outimg.data[(i*Outimg.cols*3)+(j*3)+0] = gray;
            Outimg.data[(i*Outimg.cols*3)+(j*3)+1] = gray;
            Outimg.data[(i*Outimg.cols*3)+(j*3)+2] = gray;
            gray_num[gray]++;
        }
    }
    for(int i=0;i<256;i++)gray_sum = gray_sum+(gray_num[i]*i);
    gray_avg = gray_sum/(Outimg.rows*Outimg.cols);
    avg_new = gray_avg;
    while((avg_old!=avg_new)&&(avg_old!=avg_new-1)){
        avg_old = avg_new;
        for(int i=0;i<avg_old;i++)gray_low = gray_low+(gray_num[i]*i);
        gray_low = gray_low/(Outimg.rows*Outimg.cols);
        for(int i=avg_old;i<256;i++)gray_hight = gray_hight+(gray_num[i]*i);
        gray_hight = gray_hight/(Outimg.rows*Outimg.cols);
        avg_new = (gray_low+gray_hight)/2;
    }
    for(int i=0;i<Outimg.rows;i++){
        for(int j=0;j<Outimg.cols;j++){
            if(Outimg.data[(i*Outimg.cols*3)+(j*3)+0]<avg_new){
                Outimg.data[(i*Outimg.cols*3)+(j*3)+0] = 0;
                Outimg.data[(i*Outimg.cols*3)+(j*3)+1] = 0;
                Outimg.data[(i*Outimg.cols*3)+(j*3)+2] = 0;
            }else{
                Outimg.data[(i*Outimg.cols*3)+(j*3)+0] = 255;
                Outimg.data[(i*Outimg.cols*3)+(j*3)+1] = 255;
                Outimg.data[(i*Outimg.cols*3)+(j*3)+2] = 255;
            }
        }
    }
    for(int a=0;a<360;a=a+ui->Black_Angle_->value()){
        int angle_be = a+front;
        if(angle_be>360)angle_be-=360;
        double angle_af = angle_be*M_PI/180;
        double x = cos(angle_af);
        double y = sin(angle_af);
        for(int r=inner;r<=outer;r++){
            int dis_x = x*r;
            int dis_y = y*r;
            if( Outimg.data[((center_Y-dis_y)*Outimg.cols*3)+((center_X+dis_x)*3)+0] == 0
              &&Outimg.data[((center_Y-dis_y)*Outimg.cols*3)+((center_X+dis_x)*3)+1] == 0
              &&Outimg.data[((center_Y-dis_y)*Outimg.cols*3)+((center_X+dis_x)*3)+2] == 0){
                break;
            }else{
                Outimg.data[((center_Y-dis_y)*Outimg.cols*3)+((center_X+dis_x)*3)+0] = 0;
                Outimg.data[((center_Y-dis_y)*Outimg.cols*3)+((center_X+dis_x)*3)+1] = 0;
                Outimg.data[((center_Y-dis_y)*Outimg.cols*3)+((center_X+dis_x)*3)+2] = 255;
            }
        }
    }
    for(int i=0;i<frame.rows*frame.cols*3;i++)frame.data[i] = Outimg.data[i];
    if(ui->Black_savedata->isDown()==true){
        interface->sent_blackItem(ui->Black_Gray->value(),ui->Black_Angle_->value());
        ui->Black_savedata->setDown(0);
    }
}
////////////////////////////////////////////////////////////////////////
///////////////////////////////資料顯示/////////////////////////////////////
void interface_window::Data_check(Mat frame,int center_x,int center_y){
    if(ui->checkBox->isChecked()){
        ui->fps_showlabel->setText(QString("%1").arg(interface->image_fps));
        ui->red_coordinate_showlabel->setText(QString("( %1 , %2 )").arg(interface->ball_x - center_x)
                                                           .arg(center_y - interface->ball_y));
        ui->red_LR->setText(QString(interface->ball_LR.c_str()));
        ui->red_ang_showlabel->setText(QString("%1").arg(interface->ball_ang));
        ui->red_dis_showlabel->setText(QString("%1").arg(interface->ball_dis));

        ui->blue_coordinate_showlabel->setText(QString("( %1 , %2 )").arg(interface->blue_x - center_x)
                                                             .arg(center_y - interface->blue_y));
        ui->blue_LR->setText(QString(interface->blue_LR.c_str()));
        ui->blue_ang_showlabel->setText(QString("%1").arg(interface->blue_ang));
        ui->blue_dis_showlabel->setText(QString("%1").arg(interface->blue_dis));

        ui->yellow_coordinate_showlabel->setText(QString("( %1 , %2 )").arg(interface->yellow_x - center_x)
                                                               .arg(center_y - interface->yellow_y));
        ui->yellow_LR->setText(QString(interface->yellow_LR.c_str()));
        ui->yellow_ang_showlabel->setText(QString("%1").arg(interface->yellow_ang));
        ui->yellow_dis_showlabel->setText(QString("%1").arg(interface->yellow_dis));

        frame.data[(interface->ball_y*frame.cols*3)+(interface->ball_x*3)+0] = 0;
        frame.data[(interface->ball_y*frame.cols*3)+(interface->ball_x*3)+1] = 255;
        frame.data[(interface->ball_y*frame.cols*3)+(interface->ball_x*3)+2] = 0;

        frame.data[(interface->blue_y*frame.cols*3)+(interface->blue_x*3)+0] = 0;
        frame.data[(interface->blue_y*frame.cols*3)+(interface->blue_x*3)+1] = 0;
        frame.data[(interface->blue_y*frame.cols*3)+(interface->blue_x*3)+2] = 255;

        frame.data[(interface->yellow_y*frame.cols*3)+(interface->yellow_x*3)+0] = 255;
        frame.data[(interface->yellow_y*frame.cols*3)+(interface->yellow_x*3)+1] = 0;
        frame.data[(interface->yellow_y*frame.cols*3)+(interface->yellow_x*3)+2] = 0;
    }else{
        ui->fps_showlabel->setText(QString("0"));
        ui->red_coordinate_showlabel->setText(QString("0"));
        ui->red_LR->setText(QString("NULL"));
        ui->red_ang_showlabel->setText(QString("0"));
        ui->red_dis_showlabel->setText(QString("0"));

        ui->blue_coordinate_showlabel->setText(QString("0"));
        ui->blue_LR->setText(QString("NULL"));
        ui->blue_ang_showlabel->setText(QString("0"));
        ui->blue_dis_showlabel->setText(QString("0"));

        ui->yellow_coordinate_showlabel->setText(QString("0"));
        ui->yellow_LR->setText(QString("NULL"));
        ui->yellow_ang_showlabel->setText(QString("0"));
        ui->yellow_dis_showlabel->setText(QString("0"));
    }
}
/////////////////////////////////////////////////////////////////////////
///////////////////////////////作圖//////////////////////////////////////
void interface_window::Draw_inner_outer_circle(cv::Mat frame, int center_X, int center_Y,int inner, int outer)
{
    for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
            int dis = hypot(i-center_Y,j-center_X);
            if((dis==inner)||(dis==outer)){
                frame.data[(i*frame.cols*3)+(j*3)+0] = 0;
                frame.data[(i*frame.cols*3)+(j*3)+1] = 255;
                frame.data[(i*frame.cols*3)+(j*3)+2] = 0;
            }
        }
    }
}
void interface_window::Draw_Front_Line(cv::Mat frame, int center_X, int center_Y,int inner)
{
    double angle = ui->Slider_Front->value()*M_PI/180;
    double x = cos(angle);
    double y = sin(angle);
    for(int r=0;r<inner;r++){
        int dis_x = x*r;
        int dis_y = y*r;
        frame.data[((center_Y-dis_y)*frame.cols*3)+((center_X+dis_x)*3)+0] = 0;
        frame.data[((center_Y-dis_y)*frame.cols*3)+((center_X+dis_x)*3)+1] = 255;
        frame.data[((center_Y-dis_y)*frame.cols*3)+((center_X+dis_x)*3)+2] = 255;
    }
}
/////////////////////////////////////////////////////////////////////////
