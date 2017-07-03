#include "interface_window.hpp"
#include "math.h"

#define PI 3.14159265
#define REDITEM 0x01
#define GREENITEM 0x02
#define BLUEITEM 0x04
#define YELLOWITEM 0x08
#define WHITEITEM 0x10
#define FILE_PATH "/config/HSVcolormap.bin"
#define LAUNCH_PATH "/default_config/vision_better.yaml"
#define IMAGE_TEST1 "src/vision/1.bmp"
#define initx 67
#define inity 30
using namespace std;
using namespace cv;

interface_window::interface_window(QInterface *node, QWidget *parent) :
    QMainWindow(parent),
    interface(node),
    ui(new Ui::interface_window)
{
    ui->setupUi(this);
    interface->on_init();
    startTimer(100);
    interface->init_data();

    double ang_PI;

    for(int ang=0 ; ang<360; ang++){
      ang_PI = ang*M_PI/180;
      Angle_sin.push_back(sin(ang_PI));
      Angle_cos.push_back(cos(ang_PI));
    }

    frame_counter = 0;

    interface->get_campara();
    int cam_fps;
    if(interface->camera_exposure != 0){
      cam_fps = 1 / interface->camera_exposure;
      ui->Slider_Exposure->setValue(cam_fps);
    }

    vision_path = ros::package::getPath("vision");
    fira_launch_path = ros::package::getPath("fira_launch");

/////////////////////////////////中心點前置參數////////////////////////////
    interface->get_center();
    if(interface->center_get_x != 999){
        ui->Slider_X->setValue(interface->center_get_x);
        ui->Slider_Y->setValue(interface->center_get_y);
        ui->Slider_Inner->setValue(interface->center_get_inner);
        ui->Slider_Outer->setValue(interface->center_get_outer);
        ui->Slider_Front->setValue(interface->center_get_front);
    }

    interface->get_Camera();
    if(interface->camera_High != 0){
        ui->Cam_H_Spin->setValue(interface->camera_High);
    }

    center_x = ui->Slider_X->value();
    center_y = ui->Slider_Y->value();
    center_inner = ui->Slider_Inner->value();
    center_outer = ui->Slider_Outer->value();
    center_front = ui->Slider_Front->value();
    camera_focal = camera_f(center_outer*2);
    camera_high = ui->Cam_H_Spin->value();

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////掃瞄點前置參數////////////////////////////
    interface->get_scan();
    if(!interface->scan_get_para.empty()){
      ui->Scane_NEARANGLE->setValue(interface->scan_get_para[0]);
      ui->Scane_NEARPIXEL->setValue(interface->scan_get_para[1]);
      ui->Scane_NEARGAP->setValue(interface->scan_get_para[2]);
      ui->Scane_MIDDLEGAP->setValue(interface->scan_get_para[3]);
      ui->Scane_FARGAP->setValue(interface->scan_get_para[4]);
      ui->Scane_ENDGAP->setValue(interface->scan_get_para[5]);

      ui->Dont_Search_Angle_1->setValue((interface->scan_get_para[6]+interface->scan_get_para[7])/2);
      ui->Dont_Search_Angle_2->setValue((interface->scan_get_para[8]+interface->scan_get_para[9])/2);
      ui->Dont_Search_Angle_3->setValue((interface->scan_get_para[10]+interface->scan_get_para[11])/2);
    }


//////////////////////////////////////////////////////////////////////////
/////////////////////////////////HSV前置參數//////////////////////////////
    HSV_init[0] = 360; HSV_init[1] = 0;
    HSV_init[2] = 255; HSV_init[3] = 0;
    HSV_init[4] = 255; HSV_init[5] = 0;
    interface->get_hsv();
    if(!interface->Redmap.empty() & !interface->Greenmap.empty() & !interface->Bluemap.empty() & !interface->Yellowmap.empty() & !interface->Whitemap.empty()){
        HSV_red = interface->Redmap;   HSV_green = interface->Greenmap;
        HSV_blue = interface->Bluemap; HSV_yellow = interface->Yellowmap;
        HSV_white = interface->Whitemap;
        ui->HSV_Huemax->setValue(HSV_red[0]);          ui->HSV_Huemin->setValue(HSV_red[1]);
        ui->HSV_SaturationMax->setValue(HSV_red[2]);   ui->HSV_SaturationMin->setValue(HSV_red[3]);
        ui->HSV_BrightnessMax->setValue(HSV_red[4]);   ui->HSV_BrightnessMin->setValue(HSV_red[5]);
    }else{
        for(int i=0;i<6;i++){
            HSV_red.push_back(HSV_init[i]);  HSV_green.push_back(HSV_init[i]);
            HSV_blue.push_back(HSV_init[i]); HSV_yellow.push_back(HSV_init[i]);
            HSV_white.push_back(HSV_init[i]);
        }
    }


    interface->get_blackItem();
    if(interface->black_angle!=0 || interface->black_gray!=0){
      ui->Black_Angle->setValue(interface->black_angle);
      ui->Black_Gray->setValue(interface->black_gray);
    }
/////////////////////////////////////////////////////////////////////////
}

interface_window::~interface_window()
{
    delete ui;
}
void interface_window::timerEvent(QTimerEvent *)
{
  if(ros::ok()){
    if(interface->cv_ptr != NULL){

        cv::flip(interface->cv_ptr->image, Main_frame, 1);

        double frame_HSV[Main_frame.rows*Main_frame.cols*3];
        if(ui->tabModel->currentIndex()==0){
            ui->Exposure_num->setText(QString("%1").arg(ui->Slider_Exposure->value()));
        }
        else if(ui->tabModel->currentIndex()==1){ //中心點
          Center(Main_frame);
        }
        else if(ui->tabModel->currentIndex()==2){ //掃描參數
          Scan(Main_frame);
        }
        else if(ui->tabModel->currentIndex()==3){ //物件分割
            if(ui->tabColor->currentIndex()==0){
                RGBtoHSV(Main_frame, frame_HSV);
                HSVModel(Main_frame, frame_HSV);
                HSV_PrintAngle();
                HSV_PrintBackground();
                if(ui->HSV_makefile->isDown()==true){
                    HSVmap();
                    ui->HSV_makefile->setDown(0);
                }
            }
            else if(ui->tabColor->currentIndex()==1){
                White_Line(Main_frame);
            }
            else if(ui->tabColor->currentIndex()==2){
                Black_Line(Main_frame);
            }
        }
        else if(ui->tabModel->currentIndex()==4){ //觀看視野
            Data_check(Main_frame);
        }
        dest = cvMatToQPixmap(Main_frame);
        Showimg(dest);
    }
  }
}

///////////////////////////////影像輸出////////////////////////////////////
void interface_window::Showimg(QPixmap dest){
    if(ui->check_cam->isChecked()){
        ui->Show_label->setPixmap(dest);
    }else{
        ui->Show_label->clear();
    }
}

QImage interface_window::cvMatToQImage( const cv::Mat &inMat ){
  switch ( inMat.type() )
  {
    // 8-bit, 4 channel
    case CV_8UC4:{
      QImage image( inMat.data,
                    inMat.cols, inMat.rows,
                    static_cast<int>(inMat.step),
                    QImage::Format_ARGB32 );
      return image;
    }

    // 8-bit, 3 channel
    case CV_8UC3:{
      QImage image( inMat.data,
                    inMat.cols, inMat.rows,
                    static_cast<int>(inMat.step),
                    QImage::Format_RGB888 );
      return image.rgbSwapped();
    }

    // 8-bit, 1 channel
    case CV_8UC1:{
      static QVector<QRgb>  sColorTable( 256 );
        // only create our color table the first time
        if ( sColorTable.isEmpty() ){
          for ( int i = 0; i < 256; ++i ){
            sColorTable[i] = qRgb( i, i, i );
          }
        }

      QImage image( inMat.data,
                    inMat.cols, inMat.rows,
                    static_cast<int>(inMat.step),
                    QImage::Format_Indexed8 );

      image.setColorTable( sColorTable );

      return image;
    }

    default:
      qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
      break;
  }
  return QImage();
}

QPixmap interface_window::cvMatToQPixmap( const cv::Mat &inMat){
    return QPixmap::fromImage(cvMatToQImage( inMat ));
}

/////////////////////////////////////////////////////////////////////////
///////////////////////////////攝影機/////////////////////////////////////
void interface_window::on_Exposure_send_clicked(){
  double Exposure_mm;

  Exposure_mm = 1000000 / ui->Slider_Exposure->value();

  interface->set_campara(Exposure_mm);
}

/////////////////////////////////////////////////////////////////////////
///////////////////////////////中心點/////////////////////////////////////

void interface_window::Center(Mat &frame_){
  center_x = ui->Slider_X->value();
  center_y = ui->Slider_Y->value();
  center_inner = ui->Slider_Inner->value();
  center_outer = ui->Slider_Outer->value();
  center_front = ui->Slider_Front->value();
  camera_focal = camera_f(center_outer*2);
  camera_high = ui->Cam_H_Spin->value();

  line(frame_, Point(center_x,center_y-center_inner), Point(center_x,center_y+center_inner), Scalar(0,255,0), 1);
  line(frame_, Point(center_x-center_inner,center_y), Point(center_x+center_inner,center_y), Scalar(0,255,0), 1);

  circle(frame_, Point(center_x,center_y), center_inner, Scalar(0,255,0), 0);
  circle(frame_, Point(center_x,center_y), center_outer, Scalar(0,255,0), 0);

  double d_cos = center_inner*cos(center_front*M_PI/180);
  double d_sin = center_inner*sin(center_front*M_PI/180);

  line(frame_, Point(center_x,center_y), Point(center_x+d_cos,center_y-d_sin), Scalar(0,255,255), 3);

  ui->Center_X_num->setText(QString("%1").arg(ui->Slider_X->value()));
  ui->Center_Y_num->setText(QString("%1").arg(ui->Slider_Y->value()));
  ui->Inner_num->setText(QString("%1").arg(ui->Slider_Inner->value()));
  ui->Outer_num->setText(QString("%1").arg(ui->Slider_Outer->value()));
  ui->Front_num->setText(QString("%1").arg(ui->Slider_Front->value()));

  if(ui->Center_sent->isDown()==true){
    interface->sent_center(center_x, center_y, center_inner, center_outer, center_front);
    interface->sent_Camera(camera_high,camera_focal);
    ui->Center_sent->setDown(0);
  }
}
////////////////////////////////////////////////////////////////////////
///////////////////////////////掃描參數///////////////////////////////////
int Frame_area(int num,int range){
  if(num < 0) num = 0;
  else if(num >= range) num = range-1;
  return num;
}

int Planning_angle(int ang,int angle){
  if (ang < 0) return ang+angle;
  else if (ang >= angle) return ang-angle;
  else return ang;
}

int interface_window::angle_for_distance(int dis){
  if(dis <= search_near) return search_angle;
  else if(dis > search_near && dis <= search_middle) return search_angle/2;
  else return search_angle/4;
}

void interface_window::Scan(cv::Mat &frame_){
  search_angle    = ui->Scane_NEARANGLE->value();
  search_distance = ui->Scane_NEARPIXEL->value();
  search_start    = ui->Scane_NEARGAP->value();
  search_near     = ui->Scane_MIDDLEGAP->value();
  search_middle   = ui->Scane_FARGAP->value();
  search_end      = ui->Scane_ENDGAP->value();

  dont_angle[0] = Planning_angle(ui->Dont_Search_Angle_1->value() - ui->Angle_range_1->value(),360);
  dont_angle[1] = Planning_angle(ui->Dont_Search_Angle_1->value() + ui->Angle_range_1->value(),360);
  dont_angle[2] = Planning_angle(ui->Dont_Search_Angle_2->value() - ui->Angle_range_2->value(),360);
  dont_angle[3] = Planning_angle(ui->Dont_Search_Angle_2->value() + ui->Angle_range_2->value(),360);
  dont_angle[4] = Planning_angle(ui->Dont_Search_Angle_3->value() - ui->Angle_range_2->value(),360);
  dont_angle[5] = Planning_angle(ui->Dont_Search_Angle_3->value() + ui->Angle_range_2->value(),360);

  int x,x_,y,y_;

  for(int distance = search_start ; distance <= search_end ; distance += search_distance){
    for(int angle = 0;angle < 360;){

      if(angle >= dont_angle[0] && angle <= dont_angle[1] ||
         angle >= dont_angle[2] && angle <= dont_angle[3] ||
         angle >= dont_angle[4] && angle <= dont_angle[5]) {
        angle += angle_for_distance(distance);
        continue;
      }

      x_= distance*Angle_cos[angle];
      y_= distance*Angle_sin[angle];

      x = Frame_area(center_x+x_,frame_.cols);
      y = Frame_area(center_y-y_,frame_.rows);

      frame_.data[(y*frame_.cols+x)*3+0] = 0;
      frame_.data[(y*frame_.cols+x)*3+1] = 255;
      frame_.data[(y*frame_.cols+x)*3+2] = 0;

      angle += angle_for_distance(distance);
    }
  }


  if(ui->Scane_Sent->isDown()==true){
    scan_para.push_back(ui->Scane_NEARANGLE->value());
    scan_para.push_back(ui->Scane_NEARPIXEL->value());
    scan_para.push_back(ui->Scane_NEARGAP->value());
    scan_para.push_back(ui->Scane_MIDDLEGAP->value());
    scan_para.push_back(ui->Scane_FARGAP->value());
    scan_para.push_back(ui->Scane_ENDGAP->value());
    scan_para.push_back(dont_angle[0]);
    scan_para.push_back(dont_angle[1]);
    scan_para.push_back(dont_angle[2]);
    scan_para.push_back(dont_angle[3]);
    scan_para.push_back(dont_angle[4]);
    scan_para.push_back(dont_angle[5]);
    interface->sent_scan(scan_para);
    ui->Scane_Sent->setDown(0);
  }
  scan_para.clear();
}
////////////////////////////////////////////////////////////////////////
///////////////////////////////滑鼠觸發///////////////////////////////////
void interface_window::mouseMoveEvent(QMouseEvent *event)
{
    mosue_x = event->x()-ui->frame_38->x()-ui->Slider_X->value();
    mosue_y = -1*(event->y()-15-ui->Slider_Y->value());
    Omni_distance(mosue_x,mosue_y);
}
void interface_window::mousePressEvent(QMouseEvent *event)
{
    mosue_x = event->x()-ui->frame_38->x()-ui->Slider_X->value();
    mosue_y = -1*(event->y()-15-ui->Slider_Y->value());
    Omni_distance(mosue_x,mosue_y);
}
////////////////////////////////////////////////////////////////////////
///////////////////////////////距離參數///////////////////////////////////

double interface_window::camera_f(int Omni_pixel){
  double m = (Omni_pixel*0.0099)/60;        // m = H1/H0 = D1/D0    D0 + D1 = 180
  double D0 = 180/(1+m);                    // D1 = m   *D0
  double D1 = 180/(1+1/m);                  // D0 = 1/m *D1

  double f = 1/(1/D0 + 1/D1);

  //ROS_INFO("m = %f D0 = %f D1 = %f F = %f",m,D0,D1,f);
  return D1;
}

double camera_D0(int Omni_pixel){
  double m = (Omni_pixel*0.0099)/60;        // m = H1/H0 = D1/D0    D0 + D1 = 180
  double D0 = 180/(1+m);                    // D1 = m   *D0

  //ROS_INFO("m = %f D0 = %f D1 = %f F = %f",m,D0,D1,f);
  return D0/2;
}

double interface_window::Omni_distance(int object_x , int object_y){
  double Z = -1*camera_high;
  //double c = camera_D0(outer*2);
  double c = 83.125;
  double b = c*0.8722;

  int outer = ui->Slider_Outer->value();
  camera_focal = camera_f(outer*2);

  double dis;

  double pixel_dis = sqrt(pow(object_x,2)+pow(object_y,2));

  double r = atan2(camera_focal,pixel_dis*0.0099);

  dis = Z*(pow(b,2)-pow(c,2))*cos(r) / ((pow(b,2)+pow(c,2))*sin(r) - 2*b*c);

  ROS_INFO("b = %f c = %f r=%f",b,c,r);
  ui->mouse_X->setText(QString("%1").arg(mosue_x));
  ui->mouse_Y->setText(QString("%1").arg(mosue_y));
  ui->mouse_dis->setText(QString("%1").arg(dis/10));
}

////////////////////////////////////////////////////////////////////////
///////////////////////////////RGBtoHSV///////////////////////////////////
void interface_window::RGBtoHSV(Mat &frame_, double *frame_HSV)
{
    double Rnew,Gnew,Bnew,HSVmax,HSVmin;
    int HSVnum =0;
    double H_sum,S_sum,V_sum;
    for(int i=0;i<frame_.rows;i++){
        for(int j=0;j<frame_.cols;j++){
            Bnew = frame_.data[(i*frame_.cols+j)*3+0]/255.0;
            Gnew = frame_.data[(i*frame_.cols+j)*3+1]/255.0;
            Rnew = frame_.data[(i*frame_.cols+j)*3+2]/255.0;
            RGBtoHSV_maxmin(Rnew, Gnew, Bnew, HSVmax, HSVmin);
            H_sum = RGBtoHSV_H(Rnew, Gnew, Bnew, HSVmax, HSVmin);
            S_sum = RGBtoHSV_S(HSVmax,HSVmin);
            V_sum = HSVmax*255;
            frame_HSV[HSVnum++] = H_sum;
            frame_HSV[HSVnum++] = S_sum;
            frame_HSV[HSVnum++] = V_sum;
        }
    }
}
void interface_window::RGBtoHSV_maxmin(double &Rnew, double &Gnew, double &Bnew, double &HSVmax, double &HSVmin)
{
  HSVmax = max(max(Rnew,Gnew),Bnew);
  HSVmin = min(min(Rnew,Gnew),Bnew);
}
double interface_window::RGBtoHSV_H(double Rnew, double Gnew, double Bnew, double HSVmax, double HSVmin)
{
    double range = HSVmax-HSVmin;
    double H_;
    if(range == 0){
        return 0;
    }else if(HSVmax == Rnew){
        H_ = 60.0*((Gnew-Bnew)/range);
    }else if(HSVmax == Gnew){
        H_ = 60.0*((Bnew-Rnew)/range + 2);
    }else if(HSVmax == Bnew){
        H_ = 60.0*((Rnew-Gnew)/range + 4);
    }
    if(H_ < 0) H_ += 360;
    return H_;
}
double interface_window::RGBtoHSV_S(double HSVmax, double HSVmin)
{
    double range = HSVmax-HSVmin;
    if(range == 0){
        return 0;
    }else{
        return range/HSVmax*255;
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
    }else if(ui->HSV_comboBox->currentIndex()==4){
        ui->HSV_Huemax->setValue(HSV_white[0]);          ui->HSV_Huemin->setValue(HSV_white[1]);
        ui->HSV_SaturationMax->setValue(HSV_white[2]);   ui->HSV_SaturationMin->setValue(HSV_white[3]);
        ui->HSV_BrightnessMax->setValue(HSV_white[4]);   ui->HSV_BrightnessMin->setValue(HSV_white[5]);
    }

}
void interface_window::HSVModel(Mat &frame_, double *frame_HSV)
{
    if(ui->HSV_reset->isDown() == true){
        ui->HSV_Huemax->setValue(HSV_init[0]);          ui->HSV_Huemin->setValue(HSV_init[1]);
        ui->HSV_SaturationMax->setValue(HSV_init[2]);   ui->HSV_SaturationMin->setValue(HSV_init[3]);
        ui->HSV_BrightnessMax->setValue(HSV_init[4]);   ui->HSV_BrightnessMin->setValue(HSV_init[5]);
    }

    for(int i=0;i<frame_.rows;i++){
        for(int j=0;j<frame_.cols;j++){
            if(ui->HSV_Huemin->value() < ui->HSV_Huemax->value()){
                if(   (frame_HSV[(i*frame_.cols+j)*3+0] >= ui->HSV_Huemin->value())
                    &&(frame_HSV[(i*frame_.cols+j)*3+0] <= ui->HSV_Huemax->value())
                    &&(frame_HSV[(i*frame_.cols+j)*3+1] >= ui->HSV_SaturationMin->value())
                    &&(frame_HSV[(i*frame_.cols+j)*3+1] <= ui->HSV_SaturationMax->value())
                    &&(frame_HSV[(i*frame_.cols+j)*3+2] >= ui->HSV_BrightnessMin->value())
                    &&(frame_HSV[(i*frame_.cols+j)*3+2] <= ui->HSV_BrightnessMax->value()) ){
                    if(ui->HSV_comboBox->currentIndex()==0){
                        frame_.data[(i*frame_.cols+j)*3+0] = 255;
                        frame_.data[(i*frame_.cols+j)*3+1] = 255;
                        frame_.data[(i*frame_.cols+j)*3+2] = 0;
                    }else if(ui->HSV_comboBox->currentIndex()==1){
                        frame_.data[(i*frame_.cols+j)*3+0] = 0;
                        frame_.data[(i*frame_.cols+j)*3+1] = 0;
                        frame_.data[(i*frame_.cols+j)*3+2] = 255;
                    }else if(ui->HSV_comboBox->currentIndex()==2){
                        frame_.data[(i*frame_.cols+j)*3+0] = 0;
                        frame_.data[(i*frame_.cols+j)*3+1] = 255;
                        frame_.data[(i*frame_.cols+j)*3+2] = 0;
                    }else if(ui->HSV_comboBox->currentIndex()==3){
                        frame_.data[(i*frame_.cols+j)*3+0] = 255;
                        frame_.data[(i*frame_.cols+j)*3+1] = 0;
                        frame_.data[(i*frame_.cols+j)*3+2] = 0;
                    }else if(ui->HSV_comboBox->currentIndex()==4){
                        frame_.data[(i*frame_.cols+j)*3+0] = 255;
                        frame_.data[(i*frame_.cols+j)*3+1] = 0;
                        frame_.data[(i*frame_.cols+j)*3+2] = 255;
                    }
                }
            }else{
                if(   (frame_HSV[(i*frame_.cols*3)+(j*3)+0] >= ui->HSV_Huemin->value())
                    ||(frame_HSV[(i*frame_.cols*3)+(j*3)+0] <= ui->HSV_Huemax->value())
                    &&(frame_HSV[(i*frame_.cols*3)+(j*3)+1] >= ui->HSV_SaturationMin->value())
                    &&(frame_HSV[(i*frame_.cols*3)+(j*3)+1] <= ui->HSV_SaturationMax->value())
                    &&(frame_HSV[(i*frame_.cols*3)+(j*3)+2] >= ui->HSV_BrightnessMin->value())
                    &&(frame_HSV[(i*frame_.cols*3)+(j*3)+2] <= ui->HSV_BrightnessMax->value()) ){
                    if(ui->HSV_comboBox->currentIndex()==0){
                      frame_.data[(i*frame_.cols+j)*3+0] = 255;
                      frame_.data[(i*frame_.cols+j)*3+1] = 255;
                      frame_.data[(i*frame_.cols+j)*3+2] = 0;
                    }else if(ui->HSV_comboBox->currentIndex()==1){
                      frame_.data[(i*frame_.cols+j)*3+0] = 0;
                      frame_.data[(i*frame_.cols+j)*3+1] = 0;
                      frame_.data[(i*frame_.cols+j)*3+2] = 255;
                    }else if(ui->HSV_comboBox->currentIndex()==2){
                      frame_.data[(i*frame_.cols+j)*3+0] = 0;
                      frame_.data[(i*frame_.cols+j)*3+1] = 255;
                      frame_.data[(i*frame_.cols+j)*3+2] = 0;
                    }else if(ui->HSV_comboBox->currentIndex()==3){
                      frame_.data[(i*frame_.cols+j)*3+0] = 255;
                      frame_.data[(i*frame_.cols+j)*3+1] = 0;
                      frame_.data[(i*frame_.cols+j)*3+2] = 0;
                    }else if(ui->HSV_comboBox->currentIndex()==4){
                      frame_.data[(i*frame_.cols+j)*3+0] = 255;
                      frame_.data[(i*frame_.cols+j)*3+1] = 0;
                      frame_.data[(i*frame_.cols+j)*3+2] = 255;
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
        }else if(ui->HSV_comboBox->currentIndex()==4){
            HSV_white[0] = ui->HSV_Huemax->value();         HSV_white[1] = ui->HSV_Huemin->value();
            HSV_white[2] = ui->HSV_SaturationMax->value();  HSV_white[3] = ui->HSV_SaturationMin->value();
            HSV_white[4] = ui->HSV_BrightnessMax->value();  HSV_white[5] = ui->HSV_BrightnessMin->value();
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
                if(HSV_white[1] < HSV_white[0]){
                    if( (H_sum >= HSV_white[1]) && (H_sum <= HSV_white[0])
                      &&(S_sum >= HSV_white[3]) && (S_sum <= HSV_white[2])
                      &&(V_sum >= HSV_white[5]) && (V_sum <= HSV_white[4]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | WHITEITEM;
                }else{
                    if( (H_sum >= HSV_white[1]) || (H_sum <= HSV_white[0])
                      &&(S_sum >= HSV_white[3]) && (S_sum <= HSV_white[2])
                      &&(V_sum >= HSV_white[5]) && (V_sum <= HSV_white[4]) )
                        HSVmap[r+(g<<8)+(b<<16)] = HSVmap[r+(g<<8)+(b<<16)] | WHITEITEM;
                }
            }
        }
    }
    interface->sent_hsv(HSV_red,HSV_green,HSV_blue,HSV_yellow,HSV_white);

    string Filename = vision_path+FILE_PATH;
    const char *Filename_Path = Filename.c_str();

    FILE *file=fopen(Filename_Path,"wb"); //開啟檔案來寫
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
    if(angleendpoint < anglestarpoint){
        if(anglestarpoint < 0.0)anglestarpoint += (2.0 * M_PI);
        if(angleendpoint < 0.0)angleendpoint += (2.0 * M_PI);
    }
    tmpAngle = (angleendpoint + anglestarpoint) / 2.0;
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
void interface_window::White_Line(cv::Mat &frame_)
{
    for(int i=0;i<frame_.rows;i++){
        for(int j=0;j<frame_.cols;j++){
            unsigned char gray = ( frame_.data[(i*frame_.cols*3)+(j*3)+0]
                                 + frame_.data[(i*frame_.cols*3)+(j*3)+1]
                                 + frame_.data[(i*frame_.cols*3)+(j*3)+2])/3;
            if(gray< ui->Slider_Gray->value()){
                frame_.data[(i*frame_.cols*3)+(j*3)+0] = 0;
                frame_.data[(i*frame_.cols*3)+(j*3)+1] = 0;
                frame_.data[(i*frame_.cols*3)+(j*3)+2] = 0;
            }else{
                frame_.data[(i*frame_.cols*3)+(j*3)+0] = 255;
                frame_.data[(i*frame_.cols*3)+(j*3)+1] = 255;
                frame_.data[(i*frame_.cols*3)+(j*3)+2] = 255;
            }
        }
    }
    for(int angle = 0; angle < 360; angle = angle+ui->Slider_Angle->value()){
        double x_ = Angle_cos[angle];
        double y_ = Angle_sin[angle];
        for(int r = center_inner; r <= center_outer; r++){
            int dis_x = x_*r;
            int dis_y = y_*r;
            if( frame_.data[((center_y-dis_y)*frame_.cols + center_x+dis_x)*3+0] == 255
              &&frame_.data[((center_y-dis_y)*frame_.cols + center_x+dis_x)*3+1] == 255
              &&frame_.data[((center_y-dis_y)*frame_.cols + center_x+dis_x)*3+2] == 255){
                break;
            }else{
                frame_.data[((center_y-dis_y)*frame_.cols + center_x+dis_x)*3+0] = 0;
                frame_.data[((center_y-dis_y)*frame_.cols + center_x+dis_x)*3+1] = 0;
                frame_.data[((center_y-dis_y)*frame_.cols + center_x+dis_x)*3+2] = 255;
            }
        }
    }
    if(ui->White_sent->isDown()==true){
        interface->sent_whiteline(ui->Slider_Gray->value(),ui->Slider_Angle->value());
        ui->White_sent->setDown(0);
    }
    ui->Gray_num->setText(QString("%1").arg(ui->Slider_Gray->value()));
    ui->Angle_num->setText(QString("%1").arg(ui->Slider_Angle->value()));

    line(frame_, Point(center_x,center_y-center_inner), Point(center_x,center_y+center_inner), Scalar(0,255,0), 1);
    line(frame_, Point(center_x-center_inner,center_y), Point(center_x+center_inner,center_y), Scalar(0,255,0), 1);

    circle(frame_, Point(center_x,center_y), center_inner, Scalar(0,255,0), 0);
    circle(frame_, Point(center_x,center_y), center_outer, Scalar(0,255,0), 0);
}
////////////////////////////////////////////////////////////////////////
///////////////////////////////BlackItem////////////////////////////////
void interface_window :: Black_Line(cv::Mat &frame_)
{
  for(int i=0;i<frame_.rows;i++){
      for(int j=0;j<frame_.cols;j++){
          unsigned char gray = ( frame_.data[(i*frame_.cols*3)+(j*3)+0]
                               + frame_.data[(i*frame_.cols*3)+(j*3)+1]
                               + frame_.data[(i*frame_.cols*3)+(j*3)+2])/3;
          if(gray < ui->Black_Gray->value()){
              frame_.data[(i*frame_.cols*3)+(j*3)+0] = 0;
              frame_.data[(i*frame_.cols*3)+(j*3)+1] = 0;
              frame_.data[(i*frame_.cols*3)+(j*3)+2] = 0;
          }else{
              frame_.data[(i*frame_.cols*3)+(j*3)+0] = 255;
              frame_.data[(i*frame_.cols*3)+(j*3)+1] = 255;
              frame_.data[(i*frame_.cols*3)+(j*3)+2] = 255;
          }
      }
  }
  for(int angle = 0; angle < 360; angle = angle + ui->Black_Angle->value()){
      double x_ = Angle_cos[angle];
      double y_ = Angle_sin[angle];
      for(int r = center_inner; r <= center_outer; r++){
          int dis_x = x_*r;
          int dis_y = y_*r;
          if( frame_.data[((center_y-dis_y)*frame_.cols + center_x+dis_x)*3+0] == 0
            &&frame_.data[((center_y-dis_y)*frame_.cols + center_x+dis_x)*3+1] == 0
            &&frame_.data[((center_y-dis_y)*frame_.cols + center_x+dis_x)*3+2] == 0){
              break;
          }else{
              frame_.data[((center_y-dis_y)*frame_.cols + center_x+dis_x)*3+0] = 0;
              frame_.data[((center_y-dis_y)*frame_.cols + center_x+dis_x)*3+1] = 0;
              frame_.data[((center_y-dis_y)*frame_.cols + center_x+dis_x)*3+2] = 255;
          }
      }
  }

  if(ui->Black_savedata->isDown()==true){
      interface->sent_blackItem(ui->Black_Gray->value(),ui->Black_Angle->value());
      ui->Black_savedata->setDown(0);
  }

  line(frame_, Point(center_x,center_y-center_inner), Point(center_x,center_y+center_inner), Scalar(0,255,0), 1);
  line(frame_, Point(center_x-center_inner,center_y), Point(center_x+center_inner,center_y), Scalar(0,255,0), 1);

  circle(frame_, Point(center_x,center_y), center_inner, Scalar(0,255,0), 0);
  circle(frame_, Point(center_x,center_y), center_outer, Scalar(0,255,0), 0);
}
////////////////////////////////////////////////////////////////////////
///////////////////////////////資料顯示/////////////////////////////////////
void interface_window::Data_check(Mat &frame_){
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

        if(interface->ball_x!=0)
        Draw_cross(frame_,'R');
        if(interface->blue_x!=0)
        Draw_cross(frame_,'B');
        if(interface->yellow_x!=0)
        Draw_cross(frame_,'Y');
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
/////////////////////////////////////////////////////////////////////////
void interface_window::Draw_cross(cv::Mat &frame_,char color){
    switch(color){
    case 'R':
        for(int i=-2;i<=2;i++){
            frame_.data[((interface->ball_y+i)*frame_.cols*3)+((interface->ball_x+0)*3)+0] = 0;
            frame_.data[((interface->ball_y+i)*frame_.cols*3)+((interface->ball_x+0)*3)+1] = 255;
            frame_.data[((interface->ball_y+i)*frame_.cols*3)+((interface->ball_x+0)*3)+2] = 0;
        }
        for(int j=-2;j<=2;j++){
            frame_.data[((interface->ball_y+0)*frame_.cols*3)+((interface->ball_x+j)*3)+0] = 0;
            frame_.data[((interface->ball_y+0)*frame_.cols*3)+((interface->ball_x+j)*3)+1] = 255;
            frame_.data[((interface->ball_y+0)*frame_.cols*3)+((interface->ball_x+j)*3)+2] = 0;
        }
        ui->localization_ball->setGeometry(interface->ball_x,interface->ball_y-inity,300,80);
        ui->localization_ball->setText(tr("<font color=red>R(%1 : %2)</font>").arg(interface->ball_x).arg(interface->ball_y));
    break;
    case 'B':
        for(int i=-2;i<=2;i++){
            frame_.data[((interface->blue_y+i)*frame_.cols*3)+((interface->blue_x+0)*3)+0] = 0;
            frame_.data[((interface->blue_y+i)*frame_.cols*3)+((interface->blue_x+0)*3)+1] = 0;
            frame_.data[((interface->blue_y+i)*frame_.cols*3)+((interface->blue_x+0)*3)+2] = 255;
        }
        for(int j=-2;j<=2;j++){
            frame_.data[((interface->blue_y+0)*frame_.cols*3)+((interface->blue_x+j)*3)+0] = 0;
            frame_.data[((interface->blue_y+0)*frame_.cols*3)+((interface->blue_x+j)*3)+1] = 0;
            frame_.data[((interface->blue_y+0)*frame_.cols*3)+((interface->blue_x+j)*3)+2] = 255;
        }
        ui->localization_bluedoor->setGeometry(interface->blue_x,interface->blue_y-inity,300,80);
        ui->localization_bluedoor->setText(tr("<font color=blue>B(%1 : %2)</font>").arg(interface->blue_x).arg(interface->blue_y));
    break;
    case 'Y':
        for(int i=-2;i<=2;i++){
            frame_.data[((interface->yellow_y+i)*frame_.cols*3)+((interface->yellow_x+0)*3)+0] = 255;
            frame_.data[((interface->yellow_y+i)*frame_.cols*3)+((interface->yellow_x+0)*3)+1] = 0;
            frame_.data[((interface->yellow_y+i)*frame_.cols*3)+((interface->yellow_x+0)*3)+2] = 0;
        }
        for(int j=-2;j<=2;j++){
            frame_.data[((interface->yellow_y+0)*frame_.cols*3)+((interface->yellow_x+j)*3)+0] = 255;
            frame_.data[((interface->yellow_y+0)*frame_.cols*3)+((interface->yellow_x+j)*3)+1] = 0;
            frame_.data[((interface->yellow_y+0)*frame_.cols*3)+((interface->yellow_x+j)*3)+2] = 0;
        }
        ui->localization_yellowdoor->setGeometry(interface->yellow_x,interface->yellow_y-inity,300,80);
        ui->localization_yellowdoor->setText(tr("<font color=yellow>Y(%1 : %2)</font>").arg(interface->yellow_x).arg(interface->yellow_y));
    break;
    }
}
/////////////////////////////////////////////////////////////////////////

void interface_window::on_Parameter_Dump_clicked()
{
  string Filename = "rosparam dump "+fira_launch_path+LAUNCH_PATH;
  const char *Filename_Path = Filename.c_str();
  system(Filename_Path);
}
