 #include "image_converter.hpp"
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
typedef unsigned char BYTE;

ImageConverter::ImageConverter()
   :it_(nh)
{
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
    object_pub = nh.advertise<vision::Object>("/vision/object",1);

    color_map = ColorFile();
    get_center();
    get_scan();
    get_distance();
    core_num = 0;
} 

ImageConverter::~ImageConverter()
{

}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    int StartTime = ros::Time::now().toNSec();
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //cv_ptr->image = imread( IMAGE_TEST1 , CV_LOAD_IMAGE_COLOR );
    opposite(cv_ptr->image);
    Mat Redmap(Size(cv_ptr->image.cols,cv_ptr->image.rows),CV_8UC3);
    Mat Greenmap(Size(cv_ptr->image.cols,cv_ptr->image.rows),CV_8UC3);
    Mat Bluemap(Size(cv_ptr->image.cols,cv_ptr->image.rows),CV_8UC3);
    Mat Yellowmap(Size(cv_ptr->image.cols,cv_ptr->image.rows),CV_8UC3);
    HSVmap(cv_ptr->image,Redmap,Greenmap,Bluemap,Yellowmap);
    int Redmap_x_max,Redmap_x_min,Redmap_y_max,Redmap_y_min;
    int Greenmap_x_max,Greenmap_x_min,Greenmap_y_max,Greenmap_y_min;
    int Bluemap_x_max,Bluemap_x_min,Bluemap_y_max,Bluemap_y_min;
    int Yellowmap_x_max,Yellowmap_x_min,Yellowmap_y_max,Yellowmap_y_min;
    int Red_center[2], Blue_center[2], Yellow_center[2];

    objectdet(Redmap,Redmap_x_max,Redmap_x_min,Redmap_y_max,Redmap_y_min);
    objectdet(Bluemap,Bluemap_x_max,Bluemap_x_min,Bluemap_y_max,Bluemap_y_min);
    objectdet(Yellowmap,Yellowmap_x_max,Yellowmap_x_min,Yellowmap_y_max,Yellowmap_y_min);
    place_case(Greenmap,Greenmap_x_max,Greenmap_x_min,Greenmap_y_max,Greenmap_y_min);

    Red_center[0] = (Redmap_x_max+Redmap_x_min)/2;
    Red_center[1] = (Redmap_y_max+Redmap_y_min)/2;
    Blue_center[0] = (Bluemap_x_max+Bluemap_x_min)/2;
    Blue_center[1] = (Bluemap_y_max+Bluemap_y_min)/2;
    Yellow_center[0] = (Yellowmap_x_max+Yellowmap_x_min)/2;
    Yellow_center[1] = (Yellowmap_y_max+Yellowmap_y_min)/2;

    vision::Object object_msg;
    if((Greenmap_x_max!=0) && (Greenmap_x_min!=0) && (Greenmap_y_max!=0) && (Greenmap_y_min!=0)){
        if((Redmap_x_max!=0) && (Redmap_x_min!=0) && (Redmap_y_max!=0) && (Redmap_y_min!=0)){
            if( (Red_center[0]>Greenmap_x_min) && (Red_center[0]<Greenmap_x_max)
              &&(Red_center[1]>Greenmap_y_min) && (Red_center[1]<Greenmap_y_max) ){
                objectdet_distance(Red_center[0], Red_center[1], Red_LR, Red_angle, Red_dis);
                object_msg.ball_x = Red_center[1];
                object_msg.ball_y = Red_center[0];
                object_msg.ball_LR = Red_LR;
                object_msg.ball_ang = Red_angle;
                object_msg.ball_dis = Red_dis;
            }else{
                ROS_ERROR("Can't find ball!!!");
            }
        }else{
            ROS_ERROR("Can't find ball!!!");
        }
    }else{
        ROS_ERROR("Can't find space!!!");
    }
    if((Bluemap_x_max!=0) && (Bluemap_x_min!=0) && (Bluemap_y_max!=0) && (Bluemap_y_min!=0)){
        objectdet_distance(Blue_center[0], Blue_center[1], Blue_LR, Blue_angle, Blue_dis);
        object_msg.blue_x = Blue_center[1];
        object_msg.blue_y = Blue_center[0];
        object_msg.blue_LR = Blue_LR;
        object_msg.blue_ang = Blue_angle;
        object_msg.blue_dis = Blue_dis;
    }else{
        ROS_ERROR("Can't find bluedoor!!!");
    }
    if((Yellowmap_x_max!=0) && (Yellowmap_x_min!=0) && (Yellowmap_y_max!=0) && (Yellowmap_y_min!=0)){
        objectdet_distance(Yellow_center[0], Yellow_center[1], Yellow_LR, Yellow_angle, Yellow_dis);
        object_msg.yellow_x = Yellow_center[1];
        object_msg.yellow_y = Yellow_center[0];
        object_msg.yellow_LR = Yellow_LR;
        object_msg.yellow_ang = Yellow_angle;
        object_msg.yellow_dis = Yellow_dis;
    }else{
        ROS_ERROR("Can't find yellowdoor!!!");
    }
    /////////////////////Show view/////////////////
//    if((Redmap_x_max!=0) && (Redmap_x_min!=0) && (Redmap_y_max!=0) && (Redmap_y_min!=0))
//        draw(cv_ptr->image,Redmap_x_max,Redmap_x_min,Redmap_y_max,Redmap_y_min);
//    if((Greenmap_x_max!=0) && (Greenmap_x_min!=0) && (Greenmap_y_max!=0) && (Greenmap_y_min!=0))
//        draw(cv_ptr->image,Greenmap_x_max,Greenmap_x_min,Greenmap_y_max,Greenmap_y_min);
//    if((Bluemap_x_max!=0) && (Bluemap_x_min!=0) && (Bluemap_y_max!=0) && (Bluemap_y_min!=0))
//        draw(cv_ptr->image,Bluemap_x_max,Bluemap_x_min,Bluemap_y_max,Bluemap_y_min);
//    if((Yellowmap_x_max!=0) && (Yellowmap_x_min!=0) && (Yellowmap_y_max!=0) && (Yellowmap_y_min!=0))
//        draw(cv_ptr->image,Yellowmap_x_max,Yellowmap_x_min,Yellowmap_y_max,Yellowmap_y_min);

    //cv::imshow("Image", cv_ptr->image);
    cv::waitKey(10);
    ///////////////////////////////////////////////
    /////////////////////FPS///////////////////////
    int EndTime = ros::Time::now().toNSec();
    double fps = 1000000000/(EndTime - StartTime);
    if(core_num<100){
        fps_num[core_num] = fps;
        //cout<<core_num<<endl;
        core_num++;
    }
    if(core_num==100){
        fps_avg = 0;
        for(int i=0;i<100;i++){
            fps_avg += fps_num[i];
        }
        fps_avg = fps_avg/100;
        //cout<<"FPS_avg : "<<fps_avg<<endl;
        object_msg.fps = fps_avg;
    }
    ///////////////////////////////////////////////
    object_pub.publish(object_msg);
    ros::spinOnce();
}
void ImageConverter::opposite(Mat frame){
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
vector<BYTE> ImageConverter::ColorFile()
{
    // open the file:
    streampos fileSize;
    std::ifstream file(FILE_PATH, ios::binary);
    // get its size:
    file.seekg(0, ios::end);
    fileSize = file.tellg();
    file.seekg(0, ios::beg);
    // read the data:
    vector<BYTE> fileData(fileSize);
    file.read((char*) &fileData[0], fileSize);
    return fileData;
}
void ImageConverter::HSVmap(Mat frame, Mat Redmap, Mat Greenmap, Mat Bluemap, Mat Yellowmap){
    for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
            unsigned char B = frame.data[(i*frame.cols*3)+(j*3)+0];
            unsigned char G = frame.data[(i*frame.cols*3)+(j*3)+1];
            unsigned char R = frame.data[(i*frame.cols*3)+(j*3)+2];
            if(color_map[R+(G<<8)+(B<<16)] & REDITEM){
                Redmap.data[(i*Redmap.cols*3)+(j*3)+0] = 255;
                Redmap.data[(i*Redmap.cols*3)+(j*3)+1] = 255;
                Redmap.data[(i*Redmap.cols*3)+(j*3)+2] = 255;
            }else{
                Redmap.data[(i*Redmap.cols*3)+(j*3)+0] = 0;
                Redmap.data[(i*Redmap.cols*3)+(j*3)+1] = 0;
                Redmap.data[(i*Redmap.cols*3)+(j*3)+2] = 0;
            }
            if(color_map[R+(G<<8)+(B<<16)] & GREENITEM){
                Greenmap.data[(i*Greenmap.cols*3)+(j*3)+0] = 255;
                Greenmap.data[(i*Greenmap.cols*3)+(j*3)+1] = 255;
                Greenmap.data[(i*Greenmap.cols*3)+(j*3)+2] = 255;
            }else{
                Greenmap.data[(i*Greenmap.cols*3)+(j*3)+0] = 0;
                Greenmap.data[(i*Greenmap.cols*3)+(j*3)+1] = 0;
                Greenmap.data[(i*Greenmap.cols*3)+(j*3)+2] = 0;
            }
            if(color_map[R+(G<<8)+(B<<16)] & BLUEITEM){
                Bluemap.data[(i*Bluemap.cols*3)+(j*3)+0] = 255;
                Bluemap.data[(i*Bluemap.cols*3)+(j*3)+1] = 255;
                Bluemap.data[(i*Bluemap.cols*3)+(j*3)+2] = 255;
            }else{
                Bluemap.data[(i*Bluemap.cols*3)+(j*3)+0] = 0;
                Bluemap.data[(i*Bluemap.cols*3)+(j*3)+1] = 0;
                Bluemap.data[(i*Bluemap.cols*3)+(j*3)+2] = 0;
            }
            if(color_map[R+(G<<8)+(B<<16)] & YELLOWITEM){
                Yellowmap.data[(i*Yellowmap.cols*3)+(j*3)+0] = 255;
                Yellowmap.data[(i*Yellowmap.cols*3)+(j*3)+1] = 255;
                Yellowmap.data[(i*Yellowmap.cols*3)+(j*3)+2] = 255;
            }else{
                Yellowmap.data[(i*Yellowmap.cols*3)+(j*3)+0] = 0;
                Yellowmap.data[(i*Yellowmap.cols*3)+(j*3)+1] = 0;
                Yellowmap.data[(i*Yellowmap.cols*3)+(j*3)+2] = 0;
            }
        }
    }
}
void ImageConverter::get_center(){
    nh.getParam("/FIRA/Center/X",center_x);
    nh.getParam("/FIRA/Center/Y",center_y);
    nh.getParam("/FIRA/Center/Inner",center_inner);
    nh.getParam("/FIRA/Center/Outer",center_outer);
    nh.getParam("/FIRA/Center/Front",center_front);
}
void ImageConverter::get_scan(){
    nh.getParam("/FIRA/Scan/Parameter",scan_para);
    nh.getParam("/FIRA/Scan/Near",scan_near);
    nh.getParam("/FIRA/Scan/Middle",scan_middle);
    nh.getParam("/FIRA/Scan/Far",scan_far);
}
void ImageConverter::get_distance(){
    nh.getParam("/FIRA/Distance/Gap",dis_gap);
    nh.getParam("/FIRA/Distance/Space",dis_space);
    nh.getParam("/FIRA/Distance/Pixel",dis_pixel);
}
void ImageConverter::objectdet(Mat frame, int &x_max, int &x_min, int &y_max, int &y_min){
    int neardis   = (scan_para[3]-scan_para[0])/scan_para[2]*2;
    int middledis = (scan_para[5]-scan_para[3])/scan_para[4]*2;
    int fardis    = (scan_para[7]-scan_para[5])/scan_para[6]*2;
    int nearangle   = 360/(scan_para[1]/10);
    int middleangle = nearangle*2;
    int farangle    = middleangle*2;
    deque<int>nearpoint;
    deque<int>middlepoint;
    deque<int>farpoint;
    int nearnum_max[2] = {0};//1.num_adress 2.point_num
    int middlenum_max[2] = {0};
    int farnum_max[2] = {0};
    int objectnum_max[3] = {0};//1.nmf_adress 2.num_adress 3.point_num
    objectdet_search(frame, scan_near,nearpoint, nearangle, neardis,1);
    objectdet_search(frame, scan_middle,middlepoint, middleangle, middledis,2);
    objectdet_search(frame, scan_far,farpoint, farangle, fardis,3);
    if((!near_num.empty()) || (!middle_num.empty()) || (!far_num.empty())){
        if(!near_num.empty()){
            nearnum_max[0]=0;nearnum_max[1]=near_num[0];
            for(int i=1;i<near_num.size();i++){
                if(near_num[i]>nearnum_max[1]){
                    nearnum_max[0]=i;nearnum_max[1]=near_num[i];
                }
            }
        }
        if(!middle_num.empty()){
            middlenum_max[0]=0;middlenum_max[1]=middle_num[0];
            for(int i=1;i<middle_num.size();i++){
                if(middle_num[i]>middlenum_max[1]){
                    middlenum_max[0]=i;middlenum_max[1]=middle_num[i];
                }
            }
        }
        if(!far_num.empty()){
            farnum_max[0]=0;farnum_max[1]=far_num[0];
            for(int i=1;i<far_num.size();i++){
                if(far_num[i]>farnum_max[1]){
                    farnum_max[0]=i;farnum_max[1]=far_num[i];
                }
            }
        }
        if(nearnum_max[1] > middlenum_max[1]){
            if(nearnum_max[1] > farnum_max[1]){
                objectnum_max[0] = 1;
                objectnum_max[1] = nearnum_max[0];
                objectnum_max[2] = nearnum_max[1];
            }else{
                objectnum_max[0] = 3;
                objectnum_max[1] = farnum_max[0];
                objectnum_max[2] = farnum_max[1];
            }
        }else{
            if(middlenum_max[1] > farnum_max[1]){
                objectnum_max[0] = 2;
                objectnum_max[1] = middlenum_max[0];
                objectnum_max[2] = middlenum_max[1];
            }else{
                objectnum_max[0] = 3;
                objectnum_max[1] = farnum_max[0];
                objectnum_max[2] = farnum_max[1];
            }
        }
        int point_bef = 0;
        if(objectnum_max[0]==1){
            for(int i=0;i<objectnum_max[1];i++)
                point_bef += near_num[i]*2;
            x_max = near_point[1+point_bef];
            x_min = near_point[1+point_bef];
            y_max = near_point[point_bef];
            y_min = near_point[point_bef];
            for(int i=0;i<objectnum_max[2]*2;i+=2){
                int x = near_point[i+1+point_bef];
                int y = near_point[i+point_bef];
                if(x_max<x)x_max=near_point[i+1+point_bef];
                if(x_min>x)x_min=near_point[i+1+point_bef];
                if(y_max<y)y_max=near_point[i+point_bef];
                if(y_min>y)y_min=near_point[i+point_bef];
            }
        }else if(objectnum_max[0]==2){
            for(int i=0;i<objectnum_max[1];i++)
                point_bef += middle_num[i]*2;
            x_max = middle_point[1+point_bef];
            x_min = middle_point[1+point_bef];
            y_max = middle_point[point_bef];
            y_min = middle_point[point_bef];
            for(int i=0;i<objectnum_max[2]*2;i+=2){
                int x = middle_point[i+1+point_bef];
                int y = middle_point[i+point_bef];
                if(x_max<x)x_max=middle_point[i+1+point_bef];
                if(x_min>x)x_min=middle_point[i+1+point_bef];
                if(y_max<y)y_max=middle_point[i+point_bef];
                if(y_min>y)y_min=middle_point[i+point_bef];
            }
        }else if(objectnum_max[0]==3){
            for(int i=0;i<objectnum_max[1];i++)
                point_bef += far_num[i]*2;
            x_max = far_point[1+point_bef];
            x_min = far_point[1+point_bef];
            y_max = far_point[point_bef];
            y_min = far_point[point_bef];
            for(int i=0;i<objectnum_max[2]*2;i+=2){
                int x = far_point[i+1+point_bef];
                int y = far_point[i+point_bef];
                if(x_max<x)x_max=far_point[i+1+point_bef];
                if(x_min>x)x_min=far_point[i+1+point_bef];
                if(y_max<y)y_max=far_point[i+point_bef];
                if(y_min>y)y_min=far_point[i+point_bef];
            }
        }
    }else{
        x_max = 0;x_min = 0;y_max = 0;y_min = 0;
    }
    near_num.clear();near_point.clear();
    middle_num.clear();middle_point.clear();
    far_num.clear();far_point.clear();
}
void ImageConverter::objectdet_search(Mat frame, vector<int> &scan, deque<int> &point, int angle, int distance, int level ){

    int neardis   = (scan_para[3]-scan_para[0])/scan_para[2]*2;
    int middledis = (scan_para[5]-scan_para[3])/scan_para[4]*2;
    int fardis    = (scan_para[7]-scan_para[5])/scan_para[6]*2;
    int nearangle   = 360/(scan_para[1]/10);
    int middleangle = nearangle*2;
    int farangle    = middleangle*2;

    int dis,ang;
    if(!scan.empty()){
        for(int i=0;i<angle;i++){
            for(int j=0;j<distance;j=j+2){
                num = 0;
                if(frame.data[(scan[(i*distance)+j+1]*frame.cols*3)+(scan[(i*distance)+j]*3)+0] == 255){
                    objectdet_point(frame,scan,point,distance,i,j,level);
                    num = 1;
                    while(!point.empty()){
                        dis = point.front(); point.pop_front();
                        ang = point.front(); point.pop_front();
                        objectdet_arund(frame,scan,point,angle,distance,ang,dis,level);
                        if(dis==(distance-2)){
                            if(level==1){
                                near_last.push_back(ang);
                                near_last.push_back(dis);
                            }else if(level==2){
                                middle_last.push_back(ang);
                                middle_last.push_back(dis);
                            }
                        }
                    }

                    if((level==1)&&(!near_last.empty())){
                        deque<int> neartomid_point;
                        for(int po=0;po<near_last.size();po+=2){
                            int neartomid_ang =near_last[po]*2;
                            if(frame.data[(scan_middle[(neartomid_ang*middledis)+0+1]*frame.cols*3)+(scan_middle[(neartomid_ang*middledis)+0]*3)+0]==255){
                                neartomid_point.push_back(0);
                                neartomid_point.push_back(neartomid_ang);
                            }
                        }
                        while (!neartomid_point.empty()) {
                            int ntom_dis = neartomid_point.front(); neartomid_point.pop_front();
                            int ntom_ang = neartomid_point.front(); neartomid_point.pop_front();
                            objectdet_arund(frame,scan_middle,neartomid_point,middleangle,middledis,ntom_ang,ntom_dis,1);
                            if(ntom_dis==(middledis-2)){
                                middle_last.push_back(ntom_ang);
                                middle_last.push_back(ntom_dis);
                            }
                        }
                        near_last.clear();

                        if(!middle_last.empty()){
                            deque<int> midtofar_point;
                            for(int po=0;po<middle_last.size();po+=2){
                                int midtofar_ang =middle_last[po]*2;
                                if(frame.data[(scan_far[(midtofar_ang*fardis)+0+1]*frame.cols*3)+(scan_far[(midtofar_ang*fardis)+0]*3)+0]==255){
                                    midtofar_point.push_back(0);
                                    midtofar_point.push_back(midtofar_ang);
                                }
                            }
                            while (!midtofar_point.empty()) {
                                int mtof_dis = midtofar_point.front(); midtofar_point.pop_front();
                                int mtof_ang = midtofar_point.front(); midtofar_point.pop_front();
                                objectdet_arund(frame,scan_far,midtofar_point,farangle,fardis,mtof_ang,mtof_dis,1);
                            }
                            middle_last.clear();
                        }

                    }
                    if((level==2)&&(!middle_last.empty())){
                        deque<int> midtofar_point;
                        for(int po=0;po<middle_last.size();po+=2){
                            int midtofar_ang =middle_last[po]*2;
                            if(frame.data[(scan_far[(midtofar_ang*fardis)+0+1]*frame.cols*3)+(scan_far[(midtofar_ang*fardis)+0]*3)+0]==255){
                                midtofar_point.push_back(0);
                                midtofar_point.push_back(midtofar_ang);
                            }
                        }
                        while (!midtofar_point.empty()) {
                            int mtof_dis = midtofar_point.front(); midtofar_point.pop_front();
                            int mtof_ang = midtofar_point.front(); midtofar_point.pop_front();
                            objectdet_arund(frame,scan_far,midtofar_point,farangle,fardis,mtof_ang,mtof_dis,2);
                        }
                        middle_last.clear();
                    }
                    if(level==1)near_num.push_back(num);
                    else if(level==2)middle_num.push_back(num);
                    else if(level==3)far_num.push_back(num);
                }
            }
        }
    }
}
void ImageConverter::objectdet_point(Mat frame, vector<int> &scan, deque<int> &point,int distance, int ang, int dis, int level ){
    frame.data[(scan[(ang*distance)+dis+1]*frame.cols*3)+(scan[(ang*distance)+dis]*3)+0] =0;
    frame.data[(scan[(ang*distance)+dis+1]*frame.cols*3)+(scan[(ang*distance)+dis]*3)+1] =0;
    frame.data[(scan[(ang*distance)+dis+1]*frame.cols*3)+(scan[(ang*distance)+dis]*3)+2] =0;
    point.push_back(dis);point.push_back(ang);
    if(level==1){
        near_point.push_back(scan[(ang*distance)+dis]);
        near_point.push_back(scan[(ang*distance)+dis+1]);
    }else if(level==2){
        middle_point.push_back(scan[(ang*distance)+dis]);
        middle_point.push_back(scan[(ang*distance)+dis+1]);
    }else if(level==3){
        far_point.push_back(scan[(ang*distance)+dis]);
        far_point.push_back(scan[(ang*distance)+dis+1]);
    }
    num += 1;
}
void ImageConverter::objectdet_arund(Mat frame, vector<int> &scan, deque<int> &point, int angle,int distance, int ang, int dis, int level ){
    int dis_new,ang_new;
    if(dis!=0){
        dis_new = dis-2;
        if(ang==0) ang_new = angle-1;
        else ang_new = ang-1;
        if(frame.data[(scan[(ang_new*distance)+dis_new+1]*frame.cols*3)+(scan[(ang_new*distance)+dis_new]*3)+0] ==255)//ang-1 dis-2
            objectdet_point(frame,scan,point,distance,ang_new,dis_new,level);
    }

    dis_new = dis;
    if(ang==0) ang_new = angle-1;
    else ang_new = ang-1;
    if(frame.data[(scan[(ang_new*distance)+dis_new+1]*frame.cols*3)+(scan[(ang_new*distance)+dis_new]*3)+0] ==255)//ang-1 dis+0
        objectdet_point(frame,scan,point,distance,ang_new,dis_new,level);

    if( dis!=(distance-2)){
        dis_new = dis+2;
        if(ang==0) ang_new = angle-1;
        else ang_new = ang-1;
        if(frame.data[(scan[(ang_new*distance)+dis_new+1]*frame.cols*3)+(scan[(ang_new*distance)+dis_new]*3)+0] ==255)//ang-1 dis+2
            objectdet_point(frame,scan,point,distance,ang_new,dis_new,level);
    }

    if(dis!=0){
        dis_new = dis-2;
        if( ang==(angle-1)) ang_new=0;
        else ang_new = ang+1;
        if(frame.data[(scan[(ang_new*distance)+dis_new+1]*frame.cols*3)+(scan[(ang_new*distance)+dis_new]*3)+0] ==255)//ang+1 dis-2
            objectdet_point(frame,scan,point,distance,ang_new,dis_new,level);
    }

    dis_new = dis;
    if( ang==(angle-1)) ang_new=0;
    else ang_new = ang+1;
    if(frame.data[(scan[(ang_new*distance)+dis_new+1]*frame.cols*3)+(scan[(ang_new*distance)+dis_new]*3)+0] ==255)//ang+1 dis+0
        objectdet_point(frame,scan,point,distance,ang_new,dis_new,level);

    if( dis!=(distance-2)){
        dis_new = dis+2;
        if( ang==(angle-1)) ang_new=0;
        else ang_new = ang+1;
        if(frame.data[(scan[(ang_new*distance)+dis_new+1]*frame.cols*3)+(scan[(ang_new*distance)+dis_new]*3)+0] ==255)//ang+1 dis+2
            objectdet_point(frame,scan,point,distance,ang_new,dis_new,level);
    }

    if( dis!=(distance-2)){
        dis_new = dis+2;ang_new = ang;
        if(frame.data[(scan[(ang_new*distance)+dis_new+1]*frame.cols*3)+(scan[(ang_new*distance)+dis_new]*3)+0] ==255)//ang+0 dis+2
            objectdet_point(frame,scan,point,distance,ang_new,dis_new,level);
    }

    if(dis!=0){
        dis_new = dis-2;ang_new = ang;
        if(frame.data[(scan[(ang_new*distance)+dis_new+1]*frame.cols*3)+(scan[(ang_new*distance)+dis_new]*3)+0] ==255)//ang+0 dis-2
            objectdet_point(frame,scan,point,distance,ang_new,dis_new,level);

    }
}
void ImageConverter::draw(Mat frame, int x_max, int x_min, int y_max, int y_min){
    for(int i=x_min;i<=x_max;i++){
        frame.data[(i*frame.cols*3)+(y_min*3)+0] = 0;
        frame.data[(i*frame.cols*3)+(y_min*3)+1] = 255;
        frame.data[(i*frame.cols*3)+(y_min*3)+2] = 0;
        frame.data[(i*frame.cols*3)+(y_max*3)+0] = 0;
        frame.data[(i*frame.cols*3)+(y_max*3)+1] = 255;
        frame.data[(i*frame.cols*3)+(y_max*3)+2] = 0;
    }
    for(int i=y_min;i<=y_max;i++){
        frame.data[(x_min*frame.cols*3)+(i*3)+0] = 0;
        frame.data[(x_min*frame.cols*3)+(i*3)+1] = 255;
        frame.data[(x_min*frame.cols*3)+(i*3)+2] = 0;
        frame.data[(x_max*frame.cols*3)+(i*3)+0] = 0;
        frame.data[(x_max*frame.cols*3)+(i*3)+1] = 255;
        frame.data[(x_max*frame.cols*3)+(i*3)+2] = 0;
    }
}
void ImageConverter::place_case(Mat frame, int &x_max, int &x_min, int &y_max, int &y_min){
    vector<int> place_point;
    for(int i=0;i<scan_near.size();i+=2){
        if(frame.data[(scan_near[i+1]*frame.cols*3)+(scan_near[i]*3)+0] == 255){
            place_point.push_back(scan_near[i]);
            place_point.push_back(scan_near[i+1]);
        }
    }
    for(int i=0;i<scan_middle.size();i+=2){
        if(frame.data[(scan_middle[i+1]*frame.cols*3)+(scan_middle[i]*3)+0] == 255){
            place_point.push_back(scan_middle[i]);
            place_point.push_back(scan_middle[i+1]);
        }
    }
    for(int i=0;i<scan_far.size();i+=2){
        if(frame.data[(scan_far[i+1]*frame.cols*3)+(scan_far[i]*3)+0] == 255){
            place_point.push_back(scan_far[i]);
            place_point.push_back(scan_far[i+1]);
        }
    }
    if(!place_point.empty()){
        x_max = place_point[1]; x_min = place_point[1];
        y_max = place_point[0]; y_min = place_point[0];
        for(int i=0;i<place_point.size();i+=2){
            if(x_max<place_point[i+1])x_max=place_point[i+1];
            if(x_min>place_point[i+1])x_min=place_point[i+1];
            if(y_max<place_point[i])y_max=place_point[i];
            if(y_min>place_point[i])y_min=place_point[i];
        }
    }else{
        x_max = 0;x_min = 0;y_max = 0;y_min = 0;
    }
}
void ImageConverter::objectdet_color(Mat frame, Mat Redmap, Mat Greenmap, Mat Bluemap, Mat Yellowmap )
{
    for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
            if(Redmap.data[(i*frame.cols*3)+(j*3)+0] != 0){
				frame.data[(i*frame.cols*3)+(j*3)+0] = 0;
                frame.data[(i*frame.cols*3)+(j*3)+1] = 0;
                frame.data[(i*frame.cols*3)+(j*3)+2] = 255;
			}
		}
	}
	for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
			if(Greenmap.data[(i*frame.cols*3)+(j*3)+0] != 0){
				frame.data[(i*frame.cols*3)+(j*3)+0] = 0;
                frame.data[(i*frame.cols*3)+(j*3)+1] = 255;
                frame.data[(i*frame.cols*3)+(j*3)+2] = 0;
			}
		}
	}
	for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
			if(Bluemap.data[(i*frame.cols*3)+(j*3)+0] != 0){
				frame.data[(i*frame.cols*3)+(j*3)+0] = 255;
                frame.data[(i*frame.cols*3)+(j*3)+1] = 0;
                frame.data[(i*frame.cols*3)+(j*3)+2] = 0;
			}
		}
	}
	for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
			if(Yellowmap.data[(i*frame.cols*3)+(j*3)+0] != 0){
				frame.data[(i*frame.cols*3)+(j*3)+0] = 0;
                frame.data[(i*frame.cols*3)+(j*3)+1] = 255;
                frame.data[(i*frame.cols*3)+(j*3)+2] = 255;
			}
		}
    }
}
void ImageConverter::objectdet_distance(int object_x, int object_y, string &object_LR, int &objct_angle_space, int &object_dis){
    object_x = center_y - object_x;
    object_y -= center_x;
    int Dis=hypot(abs(object_x),abs(object_y));
    int Dis_sm,Dis_bi,dis_num;
    int object_ang = atan2(object_x,object_y)*180/PI;
    if(object_ang<0)object_ang+=360;
    double dis_ratio;
    Dis_sm = dis_pixel[0];
    Dis_bi = dis_pixel[0];
    if(Dis>dis_pixel[dis_pixel.size()-1]){
        object_dis = dis_space[dis_space.size()-1];
    }else{
        for(int i=1;i<dis_pixel.size();i++){
            if(dis_pixel[i]<Dis){
                dis_num = i;
                Dis_sm = dis_pixel[i];
                Dis_bi = dis_pixel[i];
            }
            if(dis_pixel[i]>Dis){
                dis_num = i;
                Dis_bi = dis_pixel[i];
                break;
            }
        }
        if(Dis == dis_pixel[dis_num-1]){
            object_dis = dis_space[dis_num-1];
        }else if(Dis == dis_pixel[dis_num]){
            object_dis = dis_space[dis_num];
        }else{
            dis_ratio = (double)(Dis-Dis_sm)/(double)(Dis_bi-Dis_sm)*(double)dis_gap;
            object_dis = dis_space[dis_num-1] + (int)dis_ratio;
        }
    }
    if(center_front<=180){
        if(object_ang>(center_front+180)){
            object_LR = "Right";
            objct_angle_space = -1*(360 - object_ang + center_front);
        }else if(object_ang<center_front){
            object_LR = "Right";
            objct_angle_space = -1*(center_front - object_ang);
        }else{
            object_LR = "Left";
            objct_angle_space = object_ang - center_front;
        }
    }else{
        if(object_ang<(center_front-180)){
            object_LR = "Left";
            objct_angle_space = object_ang + (360-center_front);
        }else if(object_ang>center_front){
            object_LR = "Left";
            objct_angle_space = object_ang - center_front;
        }else{
            object_LR = "Right";
            objct_angle_space = -1*(center_front - object_ang);
        }
    }
}
