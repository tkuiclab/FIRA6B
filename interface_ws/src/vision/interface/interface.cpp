#define PI 3.14159265
#include "interface.hpp"
#include "math.h"
#define FRAME_COLS 695 //width  x695
#define FRAME_ROWS 493 //height y493
#define IMAGE_TEST1 "/home/testa/image_transport_ws/src/interface_ws/vision/1.jpg"//圖片路徑
static const std::string OPENCV_WINDOW = "Image window";
using namespace std;
namespace enc = sensor_msgs::image_encodings;
const double ALPHA = 0.5;

void onMouse(int Event,int x,int y,int flags,void* param);
int mousex=-1;
int mousey=-1,onclick=0;
void InterfaceProc::ParameterButtonCall (const vision::parameterbutton msg)
{
  buttonmsg=msg.button;
  std::cout<<buttonmsg<<std::endl;
}
void InterfaceProc::colorcall(const vision::color msg)
{
  ColorModeMsg=msg.ColorMode;
  switch(ColorModeMsg)
  {
  case 0:
    for(int i=0;i<6;i++)BallHSVBoxMsg[i]=msg.BallHSVBox[i];
    break;
  case 1:
    for(int i=0;i<6;i++)GreenHSVBoxMsg[i]=msg.GreenHSVBox[i];
    break;
  case 2:
    for(int i=0;i<6;i++)BlueHSVBoxMsg[i]=msg.BlueHSVBox[i];
    break;
  case 3:
    for(int i=0;i<6;i++)YellowHSVBoxMsg[i]=msg.YellowHSVBox[i];
    break;
  case 4:
    for(int i=0;i<6;i++)WhiteHSVBoxMsg[i]=msg.WhiteHSVBox[i];
    break;
  }
  //std::cout<<BallHSVBoxMsg[3]<<std::endl;
}
void InterfaceProc::centercall(const vision::center msg)
{
  CenterXMsg=msg.CenterX;
  CenterYMsg=msg.CenterY;
  InnerMsg=msg.Inner;
  OuterMsg=msg.Outer;
  FrontMsg=msg.Front;
  Camera_HighMsg=msg.Camera_High;
}
void InterfaceProc::whitecall(const vision::white msg)
{
  WhiteGrayMsg=msg.Gray;
  WhiteAngleMsg=msg.Angle;
}
void InterfaceProc::cameracall(const vision::camera msg)
{
  fpsMsg=msg.fps;
}
void InterfaceProc::blackcall(const vision::black msg)
{
  BlackGrayMsg=msg.Gray;
  BlackAngleMsg=msg.Angle;
}
void InterfaceProc::colorbuttoncall(const vision::colorbutton msg)
{
  colorbottonMsg=msg.button;
}
void InterfaceProc::scancall(const vision::scan msg)
{
  Angle_Near_GapMsg=msg.Angle_Near_Gap;
  Magn_Near_GapMsg=msg.Magn_Near_Gap;
  Magn_Near_StartMsg=msg.Magn_Near_Start;
  Magn_Middle_StartMsg=msg.Magn_Middle_Start;
  Magn_Far_StartMsg=msg.Magn_Far_Start;
  Magn_Far_EndMsg=msg.Magn_Far_End;
  Dont_Search_Angle_1Msg=msg.Dont_Search_Angle_1;
  Dont_Search_Angle_2Msg=msg.Dont_Search_Angle_2;
  Dont_Search_Angle_3Msg=msg.Dont_Search_Angle_3;
  Angle_range_1Msg=msg.Angle_range_1;
  Angle_range_2_3Msg=msg.Angle_range_2_3;
}

void InterfaceProc::Parameter_getting(const int x)
{
	  if(ifstream("default.yaml")){
		system("rosparam load default.yaml");
	    nh.getParam("/FIRA/HSV/Ball",HSV_red);
		nh.getParam("/FIRA/HSV/Blue",HSV_blue);
		nh.getParam("/FIRA/HSV/Yellow",HSV_yellow);
		nh.getParam("/FIRA/HSV/Green",HSV_green);
		nh.getParam("/FIRA/HSV/white/gray",WhiteGrayMsg);
		nh.getParam("/FIRA/HSV/white/angle",WhiteAngleMsg);
		nh.getParam("/FIRA/HSV/black/gray",BlackGrayMsg);
		nh.getParam("/FIRA/HSV/black/angle",BlackAngleMsg);
	/////////////////////////////////掃瞄點前置參數///////////////////////////////////
		  nh.getParam("/FIRA/SCAN/Angle_Near_Gap",Angle_Near_GapMsg);
		  nh.getParam("/FIRA/SCAN/Magn_Near_Gap",Magn_Near_GapMsg);
		  nh.getParam("/FIRA/SCAN/Magn_Near_Start",Magn_Near_StartMsg);
		  nh.getParam("/FIRA/SCAN/Magn_Middle_Start",Magn_Middle_StartMsg);
		  nh.getParam("/FIRA/SCAN/Magn_Far_Start",Magn_Far_StartMsg);
		  nh.getParam("/FIRA/SCAN/Magn_Far_End",Magn_Far_EndMsg);
		  nh.getParam("/FIRA/SCAN/Dont_Search_Angle_1",Dont_Search_Angle_1Msg);
		  nh.getParam("/FIRA/SCAN/Dont_Search_Angle_2",Dont_Search_Angle_2Msg);
		  nh.getParam("/FIRA/SCAN/Dont_Search_Angle_3",Dont_Search_Angle_3Msg);
		  nh.getParam("/FIRA/SCAN/Angle_range_1",Angle_range_1Msg);
		  nh.getParam("/FIRA/SCAN/Angle_range_2_3",Angle_range_2_3Msg);
	///////////////////////////////////FPS設定////////////////////////////////////////////////
		nh.getParam("/FIRA/FPS",fpsMsg);
		get_campara();
	//////////////////////////////////CNETER設定///////////////////////////////////////////////
		nh.getParam("/FIRA/Center/Center_X",CenterXMsg);
		nh.getParam("/FIRA/Center/Center_Y",CenterYMsg);
		nh.getParam("/FIRA/Center/Inner",InnerMsg);
		nh.getParam("/FIRA/Center/Outer",OuterMsg);
		nh.getParam("/FIRA/Center/Front",FrontMsg);
		nh.getParam("/FIRA/Center/Camera_high",Camera_HighMsg);
	/////////////////////////////////////BUTTONMSG////////////////////////////////////////
		nh.getParam("/FIRA/Parameterbutton",buttonmsg);
	
    cout<<"read the YAML file"<<endl;

  }

  else{
    HSV_init[0] = 0; HSV_init[1] = 360;
    HSV_init[2] = 0; HSV_init[3] = 255;
    HSV_init[4] = 0; HSV_init[5] = 255;
    for(int i=0;i<6;i++){
      HSV_red.push_back(HSV_init[i]);  HSV_green.push_back(HSV_init[i]);
      HSV_blue.push_back(HSV_init[i]); HSV_yellow.push_back(HSV_init[i]);
    }
     nh.setParam("/FIRA/HSV/Ball",HSV_red);
     nh.setParam("/FIRA/HSV/Blue",HSV_blue);
     nh.setParam("/FIRA/HSV/Yellow",HSV_yellow);
     nh.setParam("/FIRA/HSV/Green",HSV_green);
     nh.setParam("/FIRA/HSV/white/gray",50);
     nh.setParam("/FIRA/HSV/white/angle",50);
     nh.setParam("/FIRA/HSV/black/gray",50);
     nh.setParam("/FIRA/HSV/black/angle",50);
/////////////////////////////////掃瞄點前置參數///////////////////////////////////
	  nh.setParam("/FIRA/SCAN/Angle_Near_Gap",20);
	  nh.setParam("/FIRA/SCAN/Magn_Near_Gap",50);
	  nh.setParam("/FIRA/SCAN/Magn_Near_Start",120);
	  nh.setParam("/FIRA/SCAN/Magn_Middle_Start",120);
	  nh.setParam("/FIRA/SCAN/Magn_Far_Start",120);
	  nh.setParam("/FIRA/SCAN/Magn_Far_End",120);
	  nh.setParam("/FIRA/SCAN/Dont_Search_Angle_1",175);
	  nh.setParam("/FIRA/SCAN/Dont_Search_Angle_2",175);
	  nh.setParam("/FIRA/SCAN/Dont_Search_Angle_3",175);
	  nh.setParam("/FIRA/SCAN/Angle_range_1",45);
	  nh.setParam("/FIRA/SCAN/Angle_range_2_3",20);
///////////////////////////////////FPS設定////////////////////////////////////////////////
    nh.setParam("/FIRA/FPS",25);
	get_campara();
//////////////////////////////////CNETER設定///////////////////////////////////////////////
    nh.setParam("/FIRA/Center/Center_X",363);
    nh.setParam("/FIRA/Center/Center_Y",363);
    nh.setParam("/FIRA/Center/Inner",363);
    nh.setParam("/FIRA/Center/Outer",363);
    nh.setParam("/FIRA/Center/Front",363);
    nh.setParam("/FIRA/Center/Camera_high",1);
	
///////////////////////////////////////////////////////////////////////////////////////////
    nh.setParam("/FIRA/Parameterbutton",2);
    system("rosparam dump Parameter.yaml");
    system("rosparam dump default.yaml");
    cout<<"Parameter is created "<<endl;
  }
	cout<<CenterXMsg<<" "<<CenterYMsg<<endl;;
}

void InterfaceProc::Parameter_setting(const vision::parametercheck msg)
{

    paraMeterCheck=msg.checkpoint;
    /*system("rosparam delete /FIRA");
    system("rosparam dump Parameter.yaml");
    remove("Parameter.yaml");*/

////////////////////////////////////如果有新的topic進來////////////////////////////
    if(paraMeterCheck!=0){

	    system("rosparam dump Parameter.yaml");
      paraMeterCheck=0;
    }
  
    
    cout<<"Parameter has change "<<endl;

}


InterfaceProc::InterfaceProc()
    :it_(nh) 
{
  ros::NodeHandle n("~");	
  Parameter_getting(1);	
  image_sub_ = it_.subscribe("/camera/image_raw", 1, &InterfaceProc::imageCb, this);
  image_pub_threshold_ = it_.advertise("/camera/image", 1);//http://localhost:8080/stream?topic=/camera/image webfor /camera/image
  s1 = nh.subscribe("interface/parameterbutton", 1000, &InterfaceProc::ParameterButtonCall, this);
  s2 = nh.subscribe("interface/color", 1000, &InterfaceProc::colorcall,this);
  s3 = nh.subscribe("interface/center", 1000, &InterfaceProc::centercall,this);
  s4 = nh.subscribe("interface/white", 1000, &InterfaceProc::whitecall,this);
  s5 = nh.subscribe("interface/camera", 1000, &InterfaceProc::cameracall,this);
  s6 = nh.subscribe("interface/black", 1000, &InterfaceProc::blackcall,this);
  s7 = nh.subscribe("interface/colorbutton", 1000, &InterfaceProc::colorbuttoncall,this);
  s8 = nh.subscribe("interface/scan", 1000, &InterfaceProc::scancall,this);
  s9 = nh.subscribe("interface/parametercheck",1000, &InterfaceProc::Parameter_setting,this);
  cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
  //cv::Mat iframe;
  frame=new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS),CV_8UC3 );
  CameraModels = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
  CenterModels = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
  ScanModels = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
  ColorModels = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
  WhiteModels = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
  BlackModels = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
  outputframe= new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
  //imshow(OPENCV_WINDOW, outputframe);
} 
InterfaceProc::~InterfaceProc()
{
    delete frame;
    delete CameraModels;
    delete CenterModels;
    delete ScanModels;
    delete ColorModels;
    delete BlackModels;
    delete WhiteModels;
    cv::destroyWindow(OPENCV_WINDOW);
}
/////////////////////////////////影像讀進來//////////////////////////////////////////
void InterfaceProc::imageCb(const sensor_msgs::ImageConstPtr& msg)
{

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  }catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  *frame = cv_ptr->image;
  *outputframe = *frame;
  //cv::imshow(OPENCV_WINDOW, *frame);
	// Image Output
  //cv::imshow(OPENCV_WINDOW, *ColorModels);
  //sensor_msgs::ImagePtr thresholdMsg = cv_bridge::CvImage(std_msgs::Header(), "mono16", *thresholdImg16).toImageMsg();
  //image_pub_threshold_.publish(thresholdMsg);
	cv::waitKey(3);

   switch(buttonmsg){
     case 1:
      *CameraModels=CameraModel(*frame);
      cv::imshow(OPENCV_WINDOW, *CameraModels);
      outputframe=CameraModels;
      break;
    case 2:
      *CenterModels=CenterModel(*frame);
	  //cout<<CenterXMsg<<endl;
      cv::imshow(OPENCV_WINDOW, *CenterModels);
      outputframe=CenterModels;
      break;
    case 3:
      *ScanModels=ScanModel(*frame);
      cv::imshow(OPENCV_WINDOW, *ScanModels);
      outputframe=ScanModels;
      break;
    case 4:
      *ColorModels =ColorModel(*frame);
      cv::imshow(OPENCV_WINDOW, *ColorModels);
      outputframe=ColorModels;
      break;
     case 5:
	  *WhiteModels =White_Line(*frame);
      cv::imshow(OPENCV_WINDOW, *WhiteModels);
      outputframe=WhiteModels;   
     case 6:
      *BlackModels =Black_Line(*frame);
      cv::imshow(OPENCV_WINDOW, *BlackModels);
      outputframe=BlackModels;
         
  }
  /*switch(colorbottonMsg){
        case 1:
           *ColorModels =ColorModel(*frame);
           cv::imshow(OPENCV_WINDOW, *ColorModels);
           outputframe=ColorModels;
           break;
        case 2:
           *WhiteModels =White_Line(*frame);
           cv::imshow(OPENCV_WINDOW, *WhiteModels);
           outputframe=WhiteModels;   
        case 3:
           *BlackModels =Black_Line(*frame);
           cv::imshow(OPENCV_WINDOW, *BlackModels);
           outputframe=BlackModels;
  }*/

  setMouseCallback(OPENCV_WINDOW, onMouse,NULL);
  if(onclick==1){
    Omni_distance(mousex-robotCenterX,mousey-robotCenterY);onclick=0;
  }
  sensor_msgs::ImagePtr thresholdMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *outputframe).toImageMsg();
 image_pub_threshold_.publish(thresholdMsg);
  cv::waitKey(3);
}
///////////////////////////////////////////////////////////////////////
////////////////////////////////ColorModel/////////////////////////////
cv::Mat InterfaceProc::ColorModel(const cv::Mat iframe)
{
  static cv::Mat oframe(cv::Size(iframe.cols,iframe.rows), CV_8UC3);
 	for (int i = 0; i < iframe.rows; i++) {
    for (int j = 0; j < iframe.cols; j++) {
	  /*c3b test = iframe.at<j,i>;
	  double B = test.val[2];
      double G = test.val[1];
      double R = test.val[0];*/

      double B = iframe.data[(i*iframe.cols*3)+(j*3)+0];//R
      double G = iframe.data[(i*iframe.cols*3)+(j*3)+1];
      double R = iframe.data[(i*iframe.cols*3)+(j*3)+2];//B
      double H,S,V;
      double Max =(max(R,G)>max(G,B))?max(R,G):max(G,B);   //max(R,G,B);
      double Min=(min(R,G)<min(G,B))?min(R,G):min(G,B);//min(R,G,B);
	  if(Max==Min)Max+=1;
      if(R==Max){H=(G-B)*60/(Max-Min);}
      if(G==Max){H=120+(B-R)*60/(Max-Min);}
      if(B==Max){H=240+(R-G)*60/(Max-Min);}
      if(B==G&&B==R) H=0;
      if(H<0){H=H+360;}
      S=(((Max-Min)*100)/Max);
      if(Max==0)S=0;
      V=Max;
      //  usleep(300);
      switch(ColorModeMsg)
      {
        case 0:
          hmax = BallHSVBoxMsg[1];
          hmin = BallHSVBoxMsg[0];
          smax = BallHSVBoxMsg[3];
          smin = BallHSVBoxMsg[2];
          vmax = BallHSVBoxMsg[5];
          vmin = BallHSVBoxMsg[4];
          break;
        case 1:
          hmax = GreenHSVBoxMsg[1];
          hmin = GreenHSVBoxMsg[0];
          smax = GreenHSVBoxMsg[3];
          smin = GreenHSVBoxMsg[2];
          vmax = GreenHSVBoxMsg[5];
          vmin = GreenHSVBoxMsg[4];
          break;
        case 2:
          hmax = BlueHSVBoxMsg[1];
          hmin = BlueHSVBoxMsg[0];
          smax = BlueHSVBoxMsg[3];
          smin = BlueHSVBoxMsg[2];
          vmax = BlueHSVBoxMsg[5];
          vmin = BlueHSVBoxMsg[4];
          break;
        case 3:
          hmax = YellowHSVBoxMsg[1];
          hmin = YellowHSVBoxMsg[0];
          smax = YellowHSVBoxMsg[3];
          smin = YellowHSVBoxMsg[2];
          vmax = YellowHSVBoxMsg[5];
          vmin = YellowHSVBoxMsg[4];
          break;
        case 4:
          hmax = WhiteHSVBoxMsg[1];
          hmin = WhiteHSVBoxMsg[0];
          smax = WhiteHSVBoxMsg[3];
          smin = WhiteHSVBoxMsg[2];
          vmax = WhiteHSVBoxMsg[5];
          vmin = WhiteHSVBoxMsg[4];
          break;
      }
      vmin=vmin*2.56;
      vmax=vmax*2.56;
      if(hmax>hmin){
        if((H<=hmax)&&(H>=hmin)&&(S<=smax)&&(S>=smin)&&(V<=vmax)&&(V>=vmin) ){
          oframe.data[(i*iframe.cols*3)+(j*3)+0] = 0;
          oframe.data[(i*iframe.cols*3)+(j*3)+1] = 0;
          oframe.data[(i*iframe.cols*3)+(j*3)+2] = 0;}else{
          oframe.data[(i*iframe.cols*3)+(j*3)+0] = iframe.data[(i*iframe.cols*3)+(j*3)+0];
          oframe.data[(i*iframe.cols*3)+(j*3)+1] = iframe.data[(i*iframe.cols*3)+(j*3)+1];
          oframe.data[(i*iframe.cols*3)+(j*3)+2] = iframe.data[(i*iframe.cols*3)+(j*3)+2];
        } 
      }else{
        if(((H<=hmax)||(H>=hmin))&&(S<=smax)&&(S>=smin)&&(V<=vmax)&&(V>=vmin) ){
          oframe.data[(i*iframe.cols*3)+(j*3)+0] = 0;
          oframe.data[(i*iframe.cols*3)+(j*3)+1] = 0;
          oframe.data[(i*iframe.cols*3)+(j*3)+2] = 0;}else{
          oframe.data[(i*iframe.cols*3)+(j*3)+0] = iframe.data[(i*iframe.cols*3)+(j*3)+0];
          oframe.data[(i*iframe.cols*3)+(j*3)+1] = iframe.data[(i*iframe.cols*3)+(j*3)+1];
          oframe.data[(i*iframe.cols*3)+(j*3)+2] = iframe.data[(i*iframe.cols*3)+(j*3)+2];
        } 
      }
    }
  }
  return oframe;
}
///////////////////////////////////////////////////////////////////////////////////
/////////////////////////FPS///////////////////////////////////////////////////
cv::Mat InterfaceProc::CameraModel(const cv::Mat iframe)
{

	//if(0<fpsMsg<=100){}else{fpsMsg=25;}
	set_campara(fpsMsg);
	return iframe;
}
///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////CenterModel//////////////////////////////////////
cv::Mat InterfaceProc::CenterModel(const cv::Mat iframe)
{
  int lengh=30,x,y;
  static cv::Mat oframe(cv::Size(iframe.cols,iframe.rows), CV_8UC3);
  oframe=iframe;
  //cout<<CenterXMsg;
  if(0<CenterXMsg<600){}else{CenterXMsg=0;CenterYMsg=0;InnerMsg=0;OuterMsg=0;FrontMsg=0;}//avoid code dump

  robotCenterX=CenterXMsg;//iframe.cols*(CenterXMsg*1);
  robotCenterY=CenterYMsg;//iframe.rows*(CenterYMsg*1);

  circle(oframe, Point(robotCenterX,robotCenterY), 1, Scalar(0,255,0), 1);
  circle(oframe, Point(robotCenterX,robotCenterY), InnerMsg , Scalar(0,0,255), 1);
  circle(oframe, Point(robotCenterX,robotCenterY), OuterMsg , Scalar(0,255,0), 1);
  x=robotCenterX+lengh*cos(FrontMsg*PI/180), y=robotCenterY-lengh*sin(FrontMsg*PI/180);
  line(oframe, Point(robotCenterX,robotCenterY), Point(x,y), Scalar(255,0,255), 1);
  return oframe;
}
double InterfaceProc::camera_f(int Omni_pixel)
{
  double m = (Omni_pixel*0.0099)/60;        // m = H1/H0 = D1/D0    D0 + D1 = 180
  double D0 = 180/(1+m);                    // D1 = m   *D0
  double D1 = 180/(1+1/m);                  // D0 = 1/m *D1

  double f = 1/(1/D0 + 1/D1);

  //ROS_INFO("m = %f D0 = %f D1 = %f F = %f",m,D0,D1,f);
  return D1;
}

///////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////distance//////////////////////////////////////////
double InterfaceProc::Omni_distance(int object_x , int object_y)
{
  double Z = -1*Camera_HighMsg;
  double c = 83.125;
  double b = c*0.8722;

  camera_focal=camera_f(OuterMsg*2);

  double dis;

  double pixel_dis = sqrt(pow(object_x,2)+pow(object_y,2));

  double r = atan2(camera_focal,pixel_dis*0.0099);

  dis = Z*(pow(b,2)-pow(c,2))*cos(r) / ((pow(b,2)+pow(c,2))*sin(r) - 2*b*c);
  //dis/=10;
  ROS_INFO("b = %f c = %f r=%f dis=%f",b,c,r,dis);
  return dis;
}
///////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////mouse////////////////////////////////////////////
void onMouse(int Event,int x,int y,int flags,void* param)
{
  if(Event==CV_EVENT_LBUTTONDOWN){
    mousex=x;
    mousey=y;
    onclick=1;
  }
}
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////掃描參數(Scan)//////////////////////////////////

//掃描點座標調整
//修正超出圖片的點座標
int Frame_Area(int coordinate, int range)
{
  if(coordinate < 0) coordinate = 0;
  else if(coordinate >= range) coordinate = range -1;
  return coordinate;
}
//角度調整
//修正大於或小於360的角度
int Angle_Adjustment(int angle)
{
  if (angle < 0) return angle + 360;
  else if (angle >= 360) return angle - 360;
  else return angle;
}
//角度間隔
//middle start 到 far start 之間　Angle near gap的值為1/2
//far start 之外 Angle near gap的值為1/4
int InterfaceProc::Angle_Interval(int radius)
{ 
  if(radius<=4){radius=4;}

  if(radius <= Magn_Middle_StartMsg) return Angle_Near_GapMsg;
  else if(radius > Magn_Middle_StartMsg && radius <= Magn_Far_StartMsg) return Angle_Near_GapMsg /2;
  else return Angle_Near_GapMsg /4;
}

cv::Mat InterfaceProc::ScanModel(const cv::Mat iframe)
{
  static cv::Mat oframe(cv::Size(iframe.cols,iframe.rows), CV_8UC3);
  oframe=iframe;
 
 if(Angle_Near_GapMsg>=4&& Angle_Near_GapMsg<=36){}else{
  Angle_Near_GapMsg=10;
  Magn_Near_GapMsg=10;
  Magn_Near_StartMsg=10;
  Magn_Middle_StartMsg=10;
  Magn_Far_StartMsg=10;
  Magn_Far_EndMsg=10;
  Dont_Search_Angle_1Msg=10;
  Dont_Search_Angle_2Msg=10;
  Dont_Search_Angle_3Msg=10;
  Angle_range_1Msg=10;
  Angle_range_2_3Msg=10;}


  int Unscaned_Area[6]={0};
  int x,y;

  Unscaned_Area[0] = Angle_Adjustment(Dont_Search_Angle_1Msg - Angle_range_1Msg);
  Unscaned_Area[1] = Angle_Adjustment(Dont_Search_Angle_1Msg + Angle_range_1Msg);
  Unscaned_Area[2] = Angle_Adjustment(Dont_Search_Angle_2Msg - Angle_range_2_3Msg);
  Unscaned_Area[3] = Angle_Adjustment(Dont_Search_Angle_2Msg + Angle_range_2_3Msg);
  Unscaned_Area[4] = Angle_Adjustment(Dont_Search_Angle_3Msg - Angle_range_2_3Msg);
  Unscaned_Area[5] = Angle_Adjustment(Dont_Search_Angle_3Msg + Angle_range_2_3Msg);


  //test
  //circle(oframe, Point(CenterXMsg,CenterYMsg), Magn_Near_StartMsg ,   Scalar(0,0,255), 1);//確認robotCenterX,robotCenterY
  //circle(oframe, Point(CenterXMsg,CenterYMsg), Magn_Middle_StartMsg , Scalar(0,255,0), 1);
  //circle(oframe, Point(CenterXMsg,CenterYMsg), Magn_Far_StartMsg ,    Scalar(0,255,0), 1);
  //circle(oframe, Point(CenterXMsg,CenterYMsg), Magn_Far_EndMsg ,      Scalar(0,255,0), 1);


  for(int radius = Magn_Near_StartMsg ; radius <= Magn_Far_EndMsg ; radius += Magn_Near_GapMsg){
    for(int angle = 0 ; angle < 360 ;){
      //略過柱子
      if(angle >= Unscaned_Area[0] && angle <= Unscaned_Area[1] ||
         angle >= Unscaned_Area[2] && angle <= Unscaned_Area[3] ||
         angle >= Unscaned_Area[4] && angle <= Unscaned_Area[5]){
        angle += Angle_Interval(radius);
        continue;
      }
      //掃描點的座標值
      x = Frame_Area(robotCenterX + radius*cos(angle*PI/180), oframe.cols);//確認CenterXMsg
      y = Frame_Area(robotCenterY - radius*sin(angle*PI/180), oframe.rows);//確認radius*sin(angle*PI/180)加減　是否圖形上下顛倒
      //畫掃描點
      oframe.data[(y*oframe.cols+x)*3+0] = 0;		//B
      oframe.data[(y*oframe.cols+x)*3+1] = 255;		//G
      oframe.data[(y*oframe.cols+x)*3+2] = 0;		//R
      angle += Angle_Interval(radius);
    }
  }
  return oframe;
}
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////White_Line///////////////////////////////
cv::Mat InterfaceProc::White_Line(const cv::Mat iframe)
{
  static cv::Mat oframe(cv::Size(iframe.cols,iframe.rows), CV_8UC3);
  oframe=iframe;
  //cout<<"111111";
  //二值化
  for(int i=0;i<oframe.rows;i++){
    for(int j=0;j<oframe.cols;j++){
      unsigned char gray = ( oframe.data[(i*oframe.cols*3)+(j*3)+0]
                           + oframe.data[(i*oframe.cols*3)+(j*3)+1]
                           + oframe.data[(i*oframe.cols*3)+(j*3)+2])/3;
      if(gray < WhiteGrayMsg){
        oframe.data[(i*oframe.cols*3)+(j*3)+0] = 0;
        oframe.data[(i*oframe.cols*3)+(j*3)+1] = 0;
        oframe.data[(i*oframe.cols*3)+(j*3)+2] = 0;
      }else{
        oframe.data[(i*oframe.cols*3)+(j*3)+0] = 255;
        oframe.data[(i*oframe.cols*3)+(j*3)+1] = 255;
        oframe.data[(i*oframe.cols*3)+(j*3)+2] = 255;
      }
    }
  }
  for(int angle = 0; angle < 360; angle = angle + WhiteAngleMsg){
    for(int r = InnerMsg; r <= OuterMsg; r++){
      int x = r*cos(angle*PI/180), y = r*sin(angle*PI/180);
      if( oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x)*3+0] == 255
        &&oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x)*3+1] == 255
        &&oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x)*3+2] == 255){
        break;
      }else{
        oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x)*3+0] = 0;
        oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x)*3+1] = 0;
        oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x)*3+2] = 255;
      }
    }
  }

  line(oframe, Point(CenterXMsg, CenterYMsg - InnerMsg), Point(CenterXMsg, CenterYMsg + InnerMsg), Scalar(0,255,0), 1);
  line(oframe, Point(CenterXMsg - InnerMsg, CenterYMsg), Point(CenterXMsg + InnerMsg, CenterYMsg), Scalar(0,255,0), 1);

  circle(oframe, Point(CenterXMsg, CenterYMsg), InnerMsg, Scalar(0,255,0), 0);
  circle(oframe, Point(CenterXMsg, CenterYMsg), OuterMsg, Scalar(0,255,0), 0);


  return oframe;
}
////////////////////////////////////////////////////////////////////////
///////////////////////////////BlackItem////////////////////////////////
cv::Mat InterfaceProc::Black_Line(const cv::Mat iframe)
{
  static cv::Mat oframe(cv::Size(iframe.cols,iframe.rows), CV_8UC3);
  oframe = iframe;

  //if(BlackGrayMsg<=100&&BlackGrayMsg>=0){}else{BlackGrayMsg=10;BlackAngleMsg=10;}
  for(int i=0;i<oframe.rows;i++){
    for(int j=0;j<oframe.cols;j++){
      unsigned char gray = ( oframe.data[(i*oframe.cols*3)+(j*3)+0]
                           + oframe.data[(i*oframe.cols*3)+(j*3)+1]
                           + oframe.data[(i*oframe.cols*3)+(j*3)+2])/3;
      if(gray < BlackGrayMsg){
        oframe.data[(i*oframe.cols*3)+(j*3)+0] = 0;
        oframe.data[(i*oframe.cols*3)+(j*3)+1] = 0;
        oframe.data[(i*oframe.cols*3)+(j*3)+2] = 0;
      }else{
        oframe.data[(i*oframe.cols*3)+(j*3)+0] = 255;
        oframe.data[(i*oframe.cols*3)+(j*3)+1] = 255;
        oframe.data[(i*oframe.cols*3)+(j*3)+2] = 255;
      }
    }
  }
  for(int angle = 0; angle < 360; angle = angle + BlackAngleMsg){
    for(int r = InnerMsg; r <= OuterMsg; r++){
      int x = r*cos(angle*PI/180), y = r*sin(angle*PI/180);
      if( oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x)*3+0] == 0
        &&oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x)*3+1] == 0
        &&oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x)*3+2] == 0){
        break;
      }else{
        oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x)*3+0] = 0;
        oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x)*3+1] = 0;
        oframe.data[((CenterYMsg - y)*oframe.cols + CenterXMsg + x)*3+2] = 255;
      }
    }
  }

  line(oframe, Point(CenterXMsg, CenterYMsg - InnerMsg), Point(CenterXMsg, CenterYMsg + InnerMsg), Scalar(0,255,0), 1);
  line(oframe, Point(CenterXMsg - InnerMsg, CenterYMsg), Point(CenterXMsg + InnerMsg, CenterYMsg), Scalar(0,255,0), 1);

  circle(oframe, Point(CenterXMsg, CenterYMsg), InnerMsg, Scalar(0,255,0), 0);
  circle(oframe, Point(CenterXMsg, CenterYMsg), OuterMsg, Scalar(0,255,0), 0);

  return oframe;
}

