#define PI 3.14159265
#include "interface.hpp"
#include "math.h"
#define FRAME_COLS 695 //width  x695
#define FRAME_ROWS 493 //height y493
#define REDITEM 0x01
#define GREENITEM 0x02
#define BLUEITEM 0x04
#define YELLOWITEM 0x08
#define WHITEITEM 0x10
#define OBSTACLEITEM 0x00
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

  center_x=msg.CenterX;
  center_y=msg.CenterY;
  center_inner=msg.Inner;
  center_outer=msg.Outer;
  center_front=msg.Front;
  Camera_H=msg.Camera_High;
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

  search_angle    = msg.Angle_Near_Gap;
  search_distance = msg.Magn_Near_Gap;
  search_start    = msg.Magn_Near_Start;
  search_near     = msg.Magn_Middle_Start;
  search_middle   = msg.Magn_Far_Start;
  search_end      = msg.Magn_Far_End;

  dont_angle[0] = msg.Dont_Search_Angle_1;
  dont_angle[1] = msg.Dont_Search_Angle_2;
  dont_angle[2] = msg.Dont_Search_Angle_3;
  dont_angle[3] = msg.Angle_range_1;
  dont_angle[4] = msg.Angle_range_2_3;
  dont_angle[5] = msg.Angle_range_2_3;
}

void InterfaceProc::Parameter_getting(const int x)
{
	  if(ifstream("default.yaml")){
		system("rosparam load default.yaml");
  }

  else{
    HSV_init[0] = 0; HSV_init[1] = 360;
    HSV_init[2] = 0; HSV_init[3] = 100;
    HSV_init[4] = 0; HSV_init[5] = 100;
    for(int i=0;i<6;i++){
      HSV_red.push_back(HSV_init[i]);  HSV_green.push_back(HSV_init[i]);
      HSV_blue.push_back(HSV_init[i]); HSV_yellow.push_back(HSV_init[i]);
    }
  nh.setParam("/FIRA/HSV/Ball",HSV_red);
  nh.setParam("/FIRA/HSV/Blue",HSV_blue);
  nh.setParam("/FIRA/HSV/Yellow",HSV_yellow);
  nh.setParam("/FIRA/HSV/Green",HSV_green);
  nh.setParam("/FIRA/HSV/ColorMode",1);
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
  nh.setParam("/FIRA/Center/Center_X",367);
  nh.setParam("/FIRA/Center/Center_Y",271);
  nh.setParam("/FIRA/Center/Inner",91);
  nh.setParam("/FIRA/Center/Outer",40);
  nh.setParam("/FIRA/Center/Front",146);
  nh.setParam("/FIRA/Center/Camera_high",1);
	
///////////////////////////////////////////////////////////////////////////////////////////
  nh.setParam("/FIRA/Parameterbutton",2);
  system("rosparam dump Parameter.yaml");
  system("rosparam dump default.yaml");
  cout<<"Parameter is created "<<endl;
  }

  nh.getParam("/FIRA/HSV/Ball",HSV_red);
  nh.getParam("/FIRA/HSV/Blue",HSV_blue);
  nh.getParam("/FIRA/HSV/Yellow",HSV_yellow);
  nh.getParam("/FIRA/HSV/Green",HSV_green);
  nh.getParam("/FIRA/HSV/ColorMode",ColorModeMsg);
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

  search_angle    = Angle_Near_GapMsg;
  search_distance = Magn_Near_GapMsg;
  search_start    = Magn_Near_StartMsg;
  search_near     = Magn_Middle_StartMsg;
  search_middle   = Magn_Far_StartMsg;
  search_end      = Magn_Far_EndMsg;

  dont_angle[0] = Dont_Search_Angle_1Msg;
  dont_angle[1] = Dont_Search_Angle_2Msg;
  dont_angle[2] = Dont_Search_Angle_3Msg;
  dont_angle[3] = Angle_range_1Msg;
  dont_angle[4] = Angle_range_2_3Msg;
  dont_angle[5] = Angle_range_2_3Msg;
///////////////////////////////////////FPS設定////////////////////////////////////////////////
	nh.getParam("/FIRA/FPS",fpsMsg);
	get_campara();
 /////// CNETER設定///////////////////////////////////////////////
  nh.getParam("/FIRA/Center/Center_X",CenterXMsg);
  nh.getParam("/FIRA/Center/Center_Y",CenterYMsg);
  nh.getParam("/FIRA/Center/Inner",InnerMsg);
  nh.getParam("/FIRA/Center/Outer",OuterMsg);
  nh.getParam("/FIRA/Center/Front",FrontMsg);
  nh.getParam("/FIRA/Center/Camera_high",Camera_HighMsg);

  center_x=CenterXMsg;
  center_y=CenterYMsg;
  center_inner=InnerMsg;
  center_outer=OuterMsg;
  center_front=FrontMsg;
  Camera_H=Camera_HighMsg;
/////////////////////////////////////BUTTONMSG////////////////////////////////////////
  nh.getParam("/FIRA/Parameterbutton",buttonmsg);
	
    cout<<"read the YAML file"<<endl;
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
  init_data();
  //image_sub_ = it_.subscribe("/camera/image_raw", 1, &InterfaceProc::imageCb, this);
  image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &InterfaceProc::imageCb, this);
  image_pub_threshold_ = it_.advertise("/camera/image", 1);//http://localhost:8080/stream?topic=/camera/image webfor /camera/image
  object_pub = nh.advertise<vision::Object>("/vision/object",1);
  CenterDis_pub = nh.advertise<vision::dis>("/interface/CenterDis",1);
  //camera_pub = nh.advertise<vision::camera>("/interface/camera_response",1);
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
   vision_path = ros::package::getPath("vision");

    color_map = ColorFile();

    double ang_PI;

    for(int ang=0 ; ang<360; ang++){
      ang_PI = ang*PI/180;
      Angle_sin.push_back(sin(ang_PI));
      Angle_cos.push_back(cos(ang_PI));
    }

  //cv::imshow(OPENCV_WINDOW, *frame);
	// Image Output
  //cv::imshow(OPENCV_WINDOW, *ColorModels);
  //sensor_msgs::ImagePtr thresholdMsg = cv_bridge::CvImage(std_msgs::Header(), "mono16", *thresholdImg16).toImageMsg();
  //image_pub_threshold_.publish(thresholdMsg);
	cv::waitKey(3);
  //cv::flip(cv_ptr->image, Main_frame, 1);
    Main_frame=cv_ptr->image.clone();
    Obstaclemap = Mat(Size(Main_frame.cols,Main_frame.rows),CV_8UC3,Scalar(0,0,0));

    object_Item_reset(Red_Item);
    object_Item_reset(Blue_Item);
    object_Item_reset(Yellow_Item);

    objectdet_change(Findmap,REDITEM,Red_Item);
    objectdet_change(Findmap,BLUEITEM,Blue_Item);
    objectdet_change(Findmap,YELLOWITEM,Yellow_Item);

    Obstacle_Item = new object_Item [5];

    object_Item_reset(Obstacle_Item[0]);
    object_Item_reset(Obstacle_Item[1]);
    object_Item_reset(Obstacle_Item[2]);
    object_Item_reset(Obstacle_Item[3]);
    object_Item_reset(Obstacle_Item[4]);
    creat_Obstclemap(Obstaclemap,OBSTACLEITEM);
    creat_FIRA_map(Obstaclemap,FIRA_map);	
    objectdet_Obstacle(Findmap,OBSTACLEITEM,Obstacle_Item);

    vision::Object object_msg;

    if(Red_Item.distance!=0){
      object_msg.ball_x = Red_Item.x;
      object_msg.ball_y = Red_Item.y;
      object_msg.ball_LR = Red_Item.LR;
      object_msg.ball_ang = Red_Item.angle;
      object_msg.ball_dis = Omni_distance_monitor(Red_Item.distance);
    }else{
      object_msg.ball_ang = 999;
      object_msg.ball_dis = 999;
    }

    if(Blue_Item.distance!=0){
      object_msg.blue_x = Blue_Item.x;
      object_msg.blue_y = Blue_Item.y;
      object_msg.blue_LR = Blue_Item.LR;
      object_msg.blue_ang = Blue_Item.angle;
      object_msg.blue_dis = Omni_distance_monitor(Blue_Item.distance);
    }else{
      object_msg.blue_ang = 999;
      object_msg.blue_dis = 999;
    }

    if(Yellow_Item.distance!=0){
      object_msg.yellow_x = Yellow_Item.x;
      object_msg.yellow_y = Yellow_Item.y;
      object_msg.yellow_LR = Yellow_Item.LR;
      object_msg.yellow_ang = Yellow_Item.angle;
      object_msg.yellow_dis = Omni_distance_monitor(Yellow_Item.distance);
    }else{
      object_msg.yellow_ang = 999;
      object_msg.yellow_dis = 999;
    }

    /////////////////////FPS///////////////////////

    frame_counter++;
    static long int StartTime = ros::Time::now().toNSec();
    static double FrameRate = 0.0;

    if(frame_counter == 10){
      EndTime = ros::Time::now().toNSec();
      dt = (EndTime - StartTime)/frame_counter;
      StartTime = EndTime;
      if( dt!=0 )
      {
              FrameRate = ( 1000000000.0 / dt ) * ALPHA + FrameRate * ( 1.0 - ALPHA );
              //cout << "FPS: " << FrameRate << endl;
      }
      frame_counter = 0;
    }
    object_msg.fps = FrameRate;
    ///////////////////////////////////////////////
    Findmap.release();
    FIRA_map.release();
    Obstaclemap.release();
    Erodemap.release();
    Dilatemap.release();
     //object_pub.publish(object_msg);
    topic_counter++;
    if(topic_counter==10){
     object_pub.publish(object_msg);
     topic_counter=0;
    }
//////////////////////處理影像開始//////////////////////////////////////
   switch(buttonmsg){
     case 1:
     // camera_pub.publish(camera_msg);
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
      break; 
     case 6:
      *BlackModels =Black_Line(*frame);
      cv::imshow(OPENCV_WINDOW, *BlackModels);
      outputframe=BlackModels;
      break;
      case 7:
      cv::imshow(OPENCV_WINDOW, Main_frame);
      *outputframe=Main_frame;
      break;
  }

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
      double Max = (max(R,G)>max(G,B))?max(R,G):max(G,B);   //max(R,G,B);
      double Min = (min(R,G)<min(G,B))?min(R,G):min(G,B);   //min(R,G,B);

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
      //smin=smin*2.56;
      //smax=smax*2.56;
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
/*
      //开操作 (去除一些噪点)
    	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
   	  morphologyEx(oframe, oframe, MORPH_OPEN, element);

   	 //闭操作 (连接一些连通域)
   	 morphologyEx(oframe, oframe, MORPH_CLOSE, element);
		Mat erodeStruct = getStructuringElement(MORPH_RECT,Size(20,20));
    erode(oframe, oframe, erodeStruct);
*/

    }
  }

	//侵蝕
	Mat Erosionomg(Size(oframe.cols,oframe.rows),CV_8UC3);
	for(int i=0;i<oframe.rows*oframe.cols*3;i++)Erosionomg.data[i] = oframe.data[i];
	for(int i=1;i<Erosionomg.rows-1;i++){
		for(int j=1;j<Erosionomg.cols-1;j++){
			if (Erosionomg.data[((i-1)*Erosionomg.cols*3)+((j-1)*3)+0] == 0
			  &&Erosionomg.data[((i-1)*Erosionomg.cols*3)+((j+0)*3)+0] == 0
			  &&Erosionomg.data[((i-1)*Erosionomg.cols*3)+((j+1)*3)+0] == 0
			  &&Erosionomg.data[((i+0)*Erosionomg.cols*3)+((j-1)*3)+0] == 0
			  &&Erosionomg.data[((i+0)*Erosionomg.cols*3)+((j+1)*3)+0] == 0
			  &&Erosionomg.data[((i+1)*Erosionomg.cols*3)+((j-1)*3)+0] == 0
			  &&Erosionomg.data[((i+1)*Erosionomg.cols*3)+((j+0)*3)+0] == 0
			  &&Erosionomg.data[((i+1)*Erosionomg.cols*3)+((j+1)*3)+0] == 0
				
				&&Erosionomg.data[((i-1)*Erosionomg.cols*3)+((j-1)*3)+1] == 0
			  &&Erosionomg.data[((i-1)*Erosionomg.cols*3)+((j+0)*3)+1] == 0
			  &&Erosionomg.data[((i-1)*Erosionomg.cols*3)+((j+1)*3)+1] == 0
			  &&Erosionomg.data[((i+0)*Erosionomg.cols*3)+((j-1)*3)+1] == 0
			  &&Erosionomg.data[((i+0)*Erosionomg.cols*3)+((j+1)*3)+1] == 0
			  &&Erosionomg.data[((i+1)*Erosionomg.cols*3)+((j-1)*3)+1] == 0
			  &&Erosionomg.data[((i+1)*Erosionomg.cols*3)+((j+0)*3)+1] == 0
			  &&Erosionomg.data[((i+1)*Erosionomg.cols*3)+((j+1)*3)+1] == 0

				&&Erosionomg.data[((i-1)*Erosionomg.cols*3)+((j-1)*3)+2] == 0
			  &&Erosionomg.data[((i-1)*Erosionomg.cols*3)+((j+0)*3)+2] == 0
			  &&Erosionomg.data[((i-1)*Erosionomg.cols*3)+((j+1)*3)+2] == 0
			  &&Erosionomg.data[((i+0)*Erosionomg.cols*3)+((j-1)*3)+2] == 0
			  &&Erosionomg.data[((i+0)*Erosionomg.cols*3)+((j+1)*3)+2] == 0
			  &&Erosionomg.data[((i+1)*Erosionomg.cols*3)+((j-1)*3)+2] == 0
			  &&Erosionomg.data[((i+1)*Erosionomg.cols*3)+((j+0)*3)+2] == 0
			  &&Erosionomg.data[((i+1)*Erosionomg.cols*3)+((j+1)*3)+2] == 0)
			{
				oframe.data[(i*oframe.cols*3)+(j*3)+0] = 0;
				oframe.data[(i*oframe.cols*3)+(j*3)+1] = 0;
		    oframe.data[(i*oframe.cols*3)+(j*3)+2] = 0;		
			}else{

			}
		}
	}


	//膨脹
	Mat Erosionomg2(Size(oframe.cols,oframe.rows),CV_8UC3);
	for(int i=0;i<oframe.rows*oframe.cols*3;i++)Erosionomg2.data[i] = oframe.data[i];
	for(int i=1;i<Erosionomg2.rows-1;i++){
		for(int j=1;j<Erosionomg2.cols-1;j++){
			if (Erosionomg2.data[((i-1)*Erosionomg2.cols*3)+((j-1)*3)+0] == 0
			  ||Erosionomg2.data[((i-1)*Erosionomg2.cols*3)+((j+0)*3)+0] == 0
			  ||Erosionomg2.data[((i-1)*Erosionomg2.cols*3)+((j+1)*3)+0] == 0
			  ||Erosionomg2.data[((i+0)*Erosionomg2.cols*3)+((j-1)*3)+0] == 0
			  ||Erosionomg2.data[((i+0)*Erosionomg2.cols*3)+((j+1)*3)+0] == 0
			  ||Erosionomg2.data[((i+1)*Erosionomg2.cols*3)+((j-1)*3)+0] == 0
			  ||Erosionomg2.data[((i+1)*Erosionomg2.cols*3)+((j+0)*3)+0] == 0
			  ||Erosionomg2.data[((i+1)*Erosionomg2.cols*3)+((j+1)*3)+0] == 0
				
				||Erosionomg2.data[((i-1)*Erosionomg2.cols*3)+((j-1)*3)+1] == 0
			  ||Erosionomg2.data[((i-1)*Erosionomg2.cols*3)+((j+0)*3)+1] == 0
			  ||Erosionomg2.data[((i-1)*Erosionomg2.cols*3)+((j+1)*3)+1] == 0
			  ||Erosionomg2.data[((i+0)*Erosionomg2.cols*3)+((j-1)*3)+1] == 0
			  ||Erosionomg2.data[((i+0)*Erosionomg2.cols*3)+((j+1)*3)+1] == 0
			  ||Erosionomg2.data[((i+1)*Erosionomg2.cols*3)+((j-1)*3)+1] == 0
			  ||Erosionomg2.data[((i+1)*Erosionomg2.cols*3)+((j+0)*3)+1] == 0
			  ||Erosionomg2.data[((i+1)*Erosionomg2.cols*3)+((j+1)*3)+1] == 0

				||Erosionomg2.data[((i-1)*Erosionomg2.cols*3)+((j-1)*3)+2] == 0
			  ||Erosionomg2.data[((i-1)*Erosionomg2.cols*3)+((j+0)*3)+2] == 0
			  ||Erosionomg2.data[((i-1)*Erosionomg2.cols*3)+((j+1)*3)+2] == 0
			  ||Erosionomg2.data[((i+0)*Erosionomg2.cols*3)+((j-1)*3)+2] == 0
			  ||Erosionomg2.data[((i+0)*Erosionomg2.cols*3)+((j+1)*3)+2] == 0
			  ||Erosionomg2.data[((i+1)*Erosionomg2.cols*3)+((j-1)*3)+2] == 0
			  ||Erosionomg2.data[((i+1)*Erosionomg2.cols*3)+((j+0)*3)+2] == 0
			  ||Erosionomg2.data[((i+1)*Erosionomg2.cols*3)+((j+1)*3)+2] == 0)
			{
				oframe.data[(i*oframe.cols*3)+(j*3)+0] = 0;
				oframe.data[(i*oframe.cols*3)+(j*3)+1] = 0;
		    oframe.data[(i*oframe.cols*3)+(j*3)+2] = 0;		
			}else{

			}
		}
	}

  return oframe;
}



///////////////////////////////////////////////////////////////////


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
  
  vision::dis dis_msg;
  dis_msg.distance=dis;
  CenterDis_pub.publish(dis_msg);
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

void InterfaceProc::object_Item_reset(object_Item &obj_){
  obj_.dis_max = 0;
  obj_.dis_min = 0;
  obj_.ang_max = 0;
  obj_.ang_min = 0;
  obj_.x = 0;
  obj_.y = 0;
  obj_.angle = 0;
  obj_.distance = 0;
  obj_.size = 0;
  obj_.LR = "null";
}
void InterfaceProc::objectdet_change(Mat &frame_, int color, object_Item &obj_item){
  int x,y;
  int x_,y_;
  int object_size;
  int dis,ang;

  frame_ = Mat(Size(Main_frame.cols,Main_frame.rows),CV_8UC3,Scalar(0,0,0));

  find_point.clear();
  object_Item_reset(FIND_Item);

  for(int distance = search_start ; distance <= search_end ; distance += search_distance){
    for(int angle = 0;angle < 360;){

      if(angle >= dont_angle[0] && angle <= dont_angle[1] ||
         angle >= dont_angle[2] && angle <= dont_angle[3] ||
         angle >= dont_angle[4] && angle <= dont_angle[5]) {
        angle += angle_for_distance(distance);
        continue;
      }
      object_size = 0;
      FIND_Item.size = 0;

      x_= distance*Angle_cos[angle];
      y_= distance*Angle_sin[angle];

      x = Frame_area(center_x+x_,frame_.cols);
      y = Frame_area(center_y-y_,frame_.rows);

      unsigned char B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
      unsigned char G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
      unsigned char R = Main_frame.data[(y*Main_frame.cols+x)*3+2];
      if(color_map[R+(G<<8)+(B<<16)] & color && frame_.data[(y*frame_.cols+x)*3+0] == 0){
        Mark_point(frame_, distance, angle ,x , y, object_size, color);
        FIND_Item.dis_max = distance;
        FIND_Item.dis_min = distance;
        FIND_Item.ang_max = angle;
        FIND_Item.ang_min = angle;
        while(!find_point.empty()){

          dis = find_point.front();
          find_point.pop_front();

          ang = find_point.front();
          find_point.pop_front();

          object_compare(dis, ang);
          find_around(frame_, dis, ang, object_size, color);
        }

        FIND_Item.size = object_size;
      }

      find_point.clear();


      if(FIND_Item.size > obj_item.size){
        obj_item = FIND_Item;
      }

      angle += angle_for_distance(distance);
    }
  }

  find_object_point(obj_item,color);

//  if(color == REDITEM){
//    draw_ellipse(frame_,Red_Item);
//    cv::imshow("R", frame_);
//    cv::waitKey(10);
//  }
//  if(color == YELLOWITEM){
//    draw_ellipse(frame_,Yellow_Item);
//    cv::imshow("Y", frame_);
//    cv::waitKey(1);
//  }
//  if(color == BLUEITEM){
//    draw_ellipse(frame_,Blue_Item);
//    cv::imshow("B", frame_);
//    cv::waitKey(10);
//  }

  draw_ellipse(Main_frame,Red_Item);
  draw_ellipse(Main_frame,Yellow_Item);
  draw_ellipse(Main_frame,Blue_Item);
     if(Red_Item.x!=0)
     Draw_cross(Main_frame,'R');
     if(Blue_Item.x!=0)
     Draw_cross(Main_frame,'B');  
     if(Yellow_Item.x!=0)
     Draw_cross(Main_frame,'Y');
}

void InterfaceProc::creat_Obstclemap(Mat &frame_, int color){
  int x,y;
  int x_,y_;
  int object_size;
  int dis,ang;

  frame_ = Mat(Size(Main_frame.cols,Main_frame.rows),CV_8UC3,Scalar(0,0,0));

  for(int distance = search_start ; distance <= search_end ; distance++){
    for(int angle = 0; angle < 360; angle++){

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

      unsigned char B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
      unsigned char G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
      unsigned char R = Main_frame.data[(y*Main_frame.cols+x)*3+2];

      if(!(color_map[R+(G<<8)+(B<<16)] ^ color)){
        Mark_point(frame_, distance, angle ,x , y, object_size, color);
      }
      angle++;
    }
  }

  cv::dilate(frame_, frame_, Mat(), Point(), 2);
  cv::erode(frame_, frame_, Mat(), Point(), 3);
  cv::dilate(frame_, frame_, Mat(), Point(), 1);

/*  if(color == OBSTACLEITEM){
    cv::imshow("obstacle_map", frame_);
    cv::waitKey(1);
  }
*/
}

void InterfaceProc::creat_FIRA_map(Mat &frame_input , Mat &frame_output){
  frame_output = Mat(Size(600,600),CV_8UC1,Scalar(0,0,0));

  int frame_center_x = frame_output.cols / 2;
  int frame_center_y = frame_output.rows / 2;

  int x,y;
  int x_,y_;

  double dis;
  int angle_;

  Point points[360];

  int distance_get = 0;

  for(int angle = 0; angle < 360; angle++){
    for(int distance = search_start ; distance <= search_middle ; distance += search_distance){
      x_= distance*Angle_cos[angle];
      y_= distance*Angle_sin[angle];

      x = Frame_area(center_x+x_,frame_input.cols);
      y = Frame_area(center_y-y_,frame_input.rows);

      if(frame_input.data[(y*frame_input.cols+x)*3+0]==255){
        distance_get = 1;

        dis = Omni_distance_monitor(distance);

        angle_ = Planning_angle(angle-center_front+90,360);

        x_= dis*Angle_cos[angle_];
        y_= dis*Angle_sin[angle_];

        x = Frame_area(frame_center_x+x_,frame_output.cols);
        y = Frame_area(frame_center_y-y_,frame_output.rows);

        points[angle] = Point(x, y);
//        frame_output.data[y*frame_output.cols+x] = 255;

        break;
      }
    }
    if(distance_get == 0){
      dis = Omni_distance_monitor(search_end);

      angle_ = Planning_angle(angle-center_front+90,360);

      x_= dis*Angle_cos[angle_];
      y_= dis*Angle_sin[angle_];

      x = Frame_area(frame_center_x+x_,frame_output.cols);
      y = Frame_area(frame_center_y-y_,frame_output.rows);

      points[angle] = Point(x, y);
//      frame_output.data[y*frame_output.cols+x] = 255;
    }
    distance_get = 0;
  }

  for(int i=0 ;i<360;i++){
    line(frame_output, points[i], points[Planning_angle(i+1,360)], 255, 1);
  }

  /*cv::imshow("FIRA_map", FIRA_map);
  cv::waitKey(1);
*/
}

void InterfaceProc::objectdet_Obstacle(Mat &frame_, int color, object_Item *obj_item){
  int x,y;
  int x_,y_;
  int object_size;
  int dis,ang;

  frame_ = Mat(Size(Main_frame.cols,Main_frame.rows),CV_8UC3,Scalar(0,0,0));

  find_point.clear();

  object_Item_reset(FIND_Item);

  for(int distance = search_start ; distance <= search_middle ; distance += search_distance){
    for(int angle = 0; angle < 360;){
      object_size = 0;
      FIND_Item.size = 0;

      x_= distance*Angle_cos[angle];
      y_= distance*Angle_sin[angle];

      x = Frame_area(center_x+x_,frame_.cols);
      y = Frame_area(center_y-y_,frame_.rows);

      unsigned char B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
      unsigned char G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
      unsigned char R = Main_frame.data[(y*Main_frame.cols+x)*3+2];

      if(Obstaclemap.data[(y*frame_.cols+x)*3+0]==255 && frame_.data[(y*frame_.cols+x)*3+0] == 0){
        Mark_point(frame_, distance, angle ,x , y, object_size, color);
        FIND_Item.dis_max = distance;
        FIND_Item.dis_min = distance;
        FIND_Item.ang_max = angle;
        FIND_Item.ang_min = angle;
        while(!find_point.empty()){
          dis = find_point.front();
          find_point.pop_front();

          ang = find_point.front();
          find_point.pop_front();

          object_compare(dis, ang);
          find_around(frame_, dis, ang, object_size, color);
        }

        FIND_Item.size = object_size;
      }

      find_point.clear();

      if(FIND_Item.size > (obj_item+0)->size && FIND_Item.size > 10){
        *(obj_item+0) = FIND_Item;
        if(FIND_Item.size > (obj_item+1)->size){
          *(obj_item+0) = *(obj_item+1);
          *(obj_item+1) = FIND_Item;
          if(FIND_Item.size > (obj_item+2)->size){
            *(obj_item+1) = *(obj_item+2);
            *(obj_item+2) = FIND_Item;
            if(FIND_Item.size > (obj_item+3)->size){
              *(obj_item+2) = *(obj_item+3);
              *(obj_item+3) = FIND_Item;
              if(FIND_Item.size > (obj_item+4)->size){
                *(obj_item+3) = *(obj_item+4);
                *(obj_item+4) = FIND_Item;
              }
            }
          }
        }
      }

      angle += angle_for_distance(distance);
    }
  }

  find_object_point(*(obj_item+0),color);
  find_object_point(*(obj_item+1),color);
  find_object_point(*(obj_item+2),color);
  find_object_point(*(obj_item+3),color);
  find_object_point(*(obj_item+4),color);

//  if(color == OBSTACLEITEM){
//    draw_ellipse(frame_,Obstacle_Item[0]);
//    draw_ellipse(frame_,Obstacle_Item[1]);
//    draw_ellipse(frame_,Obstacle_Item[2]);
//    draw_ellipse(frame_,Obstacle_Item[3]);
//    draw_ellipse(frame_,Obstacle_Item[4]);
//    cv::imshow("obstacle", frame_);
//    cv::waitKey(1);
//  }
}

void InterfaceProc::Mark_point(Mat &frame_, int distance, int angle, int x, int y, int &size, int color){
    frame_.data[(y*frame_.cols+x)*3+0] = 255;
    frame_.data[(y*frame_.cols+x)*3+1] = 255;
    frame_.data[(y*frame_.cols+x)*3+2] = 255;
    find_point.push_back(distance);
    find_point.push_back(angle);
    size += 1;
}
double InterfaceProc::Omni_distance_monitor(double dis_pixel){
  double Z = -1*Camera_H;
  double c = 83.125;
  double b = c*0.8722;

  double f = Camera_f;

  double dis;

  //double pixel_dis = sqrt(pow(object_x,2)+pow(object_y,2));

  double pixel_dis = dis_pixel;

  double r = atan2(f,pixel_dis*0.0099);

  dis = Z*(pow(b,2)-pow(c,2))*cos(r) / ((pow(b,2)+pow(c,2))*sin(r) - 2*b*c);
 /* if(dis/10 < 0 || dis/10 > 999){
    dis = 9990;
 }*/
  return dis/10;
}
void InterfaceProc::find_around(Mat &frame_, int distance ,int angle, int &size, int color){
  int x,y;
  int x_,y_;
  int dis_f,ang_f;
  double angle_f;

  for(int i=-1 ; i<2 ; i++){
    for(int j=-1 ; j<2 ; j++){
      dis_f = distance + i*search_distance;

      if(dis_f < search_start) dis_f = search_start;

      if(color == REDITEM || color == BLUEITEM || color == YELLOWITEM){
        dis_f = Frame_area(dis_f,search_end);
      }else if(color == OBSTACLEITEM){
        dis_f = Frame_area(dis_f,search_middle);
      }

      ang_f = angle + j*angle_for_distance(dis_f);

      while(Planning_angle(ang_f,360) > dont_angle[0] && Planning_angle(ang_f,360) < dont_angle[1] ||
         Planning_angle(ang_f,360) > dont_angle[2] && Planning_angle(ang_f,360) < dont_angle[3] ||
         Planning_angle(ang_f,360) > dont_angle[4] && Planning_angle(ang_f,360) < dont_angle[5]) {
        ang_f += j*angle_for_distance(dis_f);
      }

      angle_f = Planning_angle(ang_f,360);

      x_= dis_f*Angle_cos[angle_f];
      y_= dis_f*Angle_sin[angle_f];

      x = Frame_area(center_x+x_,frame_.cols);
      y = Frame_area(center_y-y_,frame_.rows);

      unsigned char B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
      unsigned char G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
      unsigned char R = Main_frame.data[(y*Main_frame.cols+x)*3+2];

      if(color == REDITEM || color == BLUEITEM || color == YELLOWITEM){
        if(color_map[R+(G<<8)+(B<<16)] & color && frame_.data[(y*frame_.cols+x)*3+0] == 0){
          Mark_point(frame_, dis_f, ang_f ,x , y, size, color);
        }
      }else if(color == OBSTACLEITEM){
        if(Obstaclemap.data[(y*frame_.cols+x)*3+0]==255 && frame_.data[(y*frame_.cols+x)*3+0] == 0){
          Mark_point(frame_, dis_f, ang_f ,x , y, size, color);
        }
      }
    }
  }
}
void InterfaceProc::find_object_point(object_Item &obj_, int color){
  int x,y;
  int x_,y_;

  int angle_,distance_;
  int angle_range;
  int find_angle;

  unsigned char B,G,R;

  if(color == REDITEM){
    angle_= Planning_angle((obj_.ang_max + obj_.ang_min)/2,360);
    distance_ = (obj_.dis_max + obj_.dis_min) / 2;

    find_angle = Planning_angle(angle_,360);

    x_= distance_*Angle_cos[find_angle];
    y_= distance_*Angle_sin[find_angle];

    x = Frame_area(center_x+x_,Main_frame.cols);
    y = Frame_area(center_y-y_,Main_frame.rows);

    obj_.x = x;
    obj_.y = y;
    obj_.distance = sqrt(pow(x_,2)+pow(y_,2));

  }else if(color == BLUEITEM || color == YELLOWITEM){
    angle_= Planning_angle((obj_.ang_max + obj_.ang_min)/2,360);
    angle_range = 0.7*Planning_angle((obj_.ang_max - obj_.ang_min)/2,360);

    for(int distance = obj_.dis_min ; distance <= obj_.dis_max ; distance++){
      for(int angle = 0 ; angle <= angle_range ; angle++){
        find_angle = Planning_angle(angle_ + angle,360);

        x_= distance*Angle_cos[find_angle];
        y_= distance*Angle_sin[find_angle];

        x = Frame_area(center_x+x_,Main_frame.cols);
        y = Frame_area(center_y-y_,Main_frame.rows);

        B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
        G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
        R = Main_frame.data[(y*Main_frame.cols+x)*3+2];

        if(color_map[R+(G<<8)+(B<<16)] & color){
          obj_.x = x;
          obj_.y = y;
          obj_.distance = sqrt(pow(x_,2)+pow(y_,2));
          angle_ = find_angle;
          break;
        }

        find_angle = Planning_angle(angle_ - angle,360);

        x_= distance*Angle_cos[find_angle];
        y_= distance*Angle_sin[find_angle];

        x = Frame_area(center_x+x_,Main_frame.cols);
        y = Frame_area(center_y-y_,Main_frame.rows);

        B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
        G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
        R = Main_frame.data[(y*Main_frame.cols+x)*3+2];

        if(color_map[R+(G<<8)+(B<<16)] & color){
          obj_.x = x;
          obj_.y = y;
          obj_.distance = sqrt(pow(x_,2)+pow(y_,2));
          angle_ = find_angle;
          break;
        }
      }
      if(obj_.distance != 0){
          break;
      }
    }
  }else if(color == OBSTACLEITEM){
    angle_= Planning_angle((obj_.ang_max + obj_.ang_min)/2,360);
    angle_range = Planning_angle((obj_.ang_max - obj_.ang_min)/2,360);

    for(int distance = obj_.dis_min ; distance <= obj_.dis_max ; distance++){
      for(int angle = 0 ; angle <= angle_range ; angle++){
        find_angle = Planning_angle(angle_ + angle,360);

        x_= distance*Angle_cos[find_angle];
        y_= distance*Angle_sin[find_angle];

        x = Frame_area(center_x+x_,Main_frame.cols);
        y = Frame_area(center_y-y_,Main_frame.rows);

        B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
        G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
        R = Main_frame.data[(y*Main_frame.cols+x)*3+2];

        if(!(color_map[R+(G<<8)+(B<<16)] ^ color)){
          obj_.x = x;
          obj_.y = y;
          obj_.distance = sqrt(pow(x_,2)+pow(y_,2));
          angle_ = find_angle;
          break;
        }

        find_angle = Planning_angle(angle_ - angle,360);

        x_= distance*Angle_cos[find_angle];
        y_= distance*Angle_sin[find_angle];

        x = Frame_area(center_x+x_,Main_frame.cols);
        y = Frame_area(center_y-y_,Main_frame.rows);

        B = Main_frame.data[(y*Main_frame.cols+x)*3+0];
        G = Main_frame.data[(y*Main_frame.cols+x)*3+1];
        R = Main_frame.data[(y*Main_frame.cols+x)*3+2];

        if(!(color_map[R+(G<<8)+(B<<16)] ^ color)){
          obj_.x = x;
          obj_.y = y;
          obj_.distance = sqrt(pow(x_,2)+pow(y_,2));
          angle_ = find_angle;
          break;
        }
      }
      if(obj_.distance != 0){
          break;
      }
    }
  }

  if(Planning_angle(angle_-center_front,360) < 180){
    obj_.LR = "Left";
    obj_.angle = Planning_angle(angle_-center_front,360);
  }else{
    obj_.LR = "Right";
    obj_.angle = Planning_angle(angle_-center_front,360)-360;
  }
}
void InterfaceProc::draw_ellipse(Mat &frame_, object_Item &obj_){
  ellipse(frame_, Point(center_x,center_y), Size(obj_.dis_min,obj_.dis_min), 0, 360-obj_.ang_max, 360-obj_.ang_min, Scalar(255,255,0), 1);
  ellipse(frame_, Point(center_x,center_y), Size(obj_.dis_max,obj_.dis_max), 0, 360-obj_.ang_max, 360-obj_.ang_min, Scalar(255,255,0), 1);
  draw_Line(frame_, obj_.dis_max, obj_.dis_min, obj_.ang_max);
  draw_Line(frame_, obj_.dis_max, obj_.dis_min, obj_.ang_min);

  circle(frame_, Point(obj_.x,obj_.y), 2, Scalar(0,0,255), -1);
}

void InterfaceProc::draw_Line(Mat &frame_, int obj_distance_max, int obj_distance_min, int obj_angle){
  int x_,y_;
  double angle_f;
  int x[2],y[2];

  angle_f = Planning_angle(obj_angle,360);

  x_= obj_distance_min*Angle_cos[angle_f];
  y_= obj_distance_min*Angle_sin[angle_f];

  x[0] = Frame_area(center_x+x_,frame_.cols);
  y[0] = Frame_area(center_y-y_,frame_.rows);

  x_= obj_distance_max*Angle_cos[angle_f];
  y_= obj_distance_max*Angle_sin[angle_f];

  x[1] = Frame_area(center_x+x_,frame_.cols);
  y[1] = Frame_area(center_y-y_,frame_.rows);

  line(frame_, Point(x[0],y[0]), Point(x[1],y[1]), Scalar(255,255,0), 1);
}
void InterfaceProc::Draw_cross(cv::Mat &frame_,char color){
    switch(color){
    case 'R':
        for(int i=-2;i<=2;i++){
            frame_.data[((Red_Item.y+i)*frame_.cols*3)+((Red_Item.x+0)*3)+0] = 0;
            frame_.data[((Red_Item.y+i)*frame_.cols*3)+((Red_Item.x+0)*3)+1] = 255;
            frame_.data[((Red_Item.y+i)*frame_.cols*3)+((Red_Item.x+0)*3)+2] = 0;
        }
        for(int j=-2;j<=2;j++){
            frame_.data[((Red_Item.y+0)*frame_.cols*3)+((Red_Item.x+j)*3)+0] = 0;
            frame_.data[((Red_Item.y+0)*frame_.cols*3)+((Red_Item.x+j)*3)+1] = 255;
            frame_.data[((Red_Item.y+0)*frame_.cols*3)+((Red_Item.x+j)*3)+2] = 0;
        }
				cv::putText(frame_, string("R"), Point(Red_Item.x,Red_Item.y), 0, 0.5, Scalar(0,0,255),1);
        //ui->localization_ball->setGeometry(Red_Item.x,Red_Item.y-inity,300,80);
        
    break;
    case 'B':
        for(int i=-2;i<=2;i++){
            frame_.data[((Blue_Item.y+i)*frame_.cols*3)+((Blue_Item.x+0)*3)+0] = 0;
            frame_.data[((Blue_Item.y+i)*frame_.cols*3)+((Blue_Item.x+0)*3)+1] = 0;
            frame_.data[((Blue_Item.y+i)*frame_.cols*3)+((Blue_Item.x+0)*3)+2] = 255;
        }
        for(int j=-2;j<=2;j++){
            frame_.data[((Blue_Item.y+0)*frame_.cols*3)+((Blue_Item.x+j)*3)+0] = 0;
            frame_.data[((Blue_Item.y+0)*frame_.cols*3)+((Blue_Item.x+j)*3)+1] = 0;
            frame_.data[((Blue_Item.y+0)*frame_.cols*3)+((Blue_Item.x+j)*3)+2] = 255;
        }
				cv::putText(frame_, string("B"), Point(Blue_Item.x,Blue_Item.y), 0, 0.5, Scalar(255,0,0),1);
       // ui->localization_bluedoor->setGeometry(Blue_Item.x,Blue_Item.y-inity,300,80);
       //ui->localization_bluedoor->setText(tr("<font color=blue>B(%1 : %2)</font>").arg(Blue_Item.x).arg(Blue_Item.y));
    break;
    case 'Y':
        for(int i=-2;i<=2;i++){
            frame_.data[((Yellow_Item.y+i)*frame_.cols*3)+((Yellow_Item.x+0)*3)+0] = 255;
            frame_.data[((Yellow_Item.y+i)*frame_.cols*3)+((Yellow_Item.x+0)*3)+1] = 0;
            frame_.data[((Yellow_Item.y+i)*frame_.cols*3)+((Yellow_Item.x+0)*3)+2] = 0;
        }
        for(int j=-2;j<=2;j++){
            frame_.data[((Yellow_Item.y+0)*frame_.cols*3)+((Yellow_Item.x+j)*3)+0] = 255;
            frame_.data[((Yellow_Item.y+0)*frame_.cols*3)+((Yellow_Item.x+j)*3)+1] = 0;
            frame_.data[((Yellow_Item.y+0)*frame_.cols*3)+((Yellow_Item.x+j)*3)+2] = 0;
        }
				cv::putText(frame_, string("Y"), Point(Yellow_Item.x,Yellow_Item.y), 0, 0.5, Scalar(0,255,255),1);
        //ui->localization_yellowdoor->setGeometry(Yellow_Item.x,Yellow_Item.y-inity,300,80);
        //ui->localization_yellowdoor->setText(tr("<font color=yellow>Y(%1 : %2)</font>").arg(Yellow_Item.x).arg(Yellow_Item.y));
    break;
    }
}
