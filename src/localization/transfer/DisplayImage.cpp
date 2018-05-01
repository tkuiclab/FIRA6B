#include "DisplayImage.hpp"
using namespace cv;

Img::Img(int argc, char **argv, const char *node_name)
{
  std::cout << "Initializing img node...\n";
  ros::init(argc, argv, node_name);
  ROS_INFO("Connected to roscore");
}
void Img::ros_comms_init()
{
  nh = new ros::NodeHandle();
  // MAP_PUB = nh->advertise<nav_msgs::OccupancyGrid>("/map", 1000);
}
void Img::map_pub()
{
  nav_msgs::OccupancyGrid map;
  ros::Time map_time = ros::Time::now();
  ros::Time map_load_time;
  double resolution = 0.01;
  int width = 750;
  int height = 550;
  // =========== translate to OccupancyGrid type =========
  map.header.frame_id = "map";
  map.header.stamp = map_time;
  map.info.map_load_time = map_load_time;
  map.info.resolution = resolution;
  map.info.width = width;
  map.info.height = height;
  // ========== geometry_msgs/Pose origin ==========
  map.info.origin.position.x = -3.75;
  map.info.origin.position.y = -2.75;
  map.info.origin.position.z = 0;
  // ========== geometry_msgs/Quaternion orientation =========
  map.info.origin.orientation.x = 0;
  map.info.origin.orientation.y = 0;
  map.info.origin.orientation.z = 0;
  map.info.origin.orientation.w = 1;
  // ========== The map data ==========
  map.data = ary;
  // printf("map size = %d",map.data.size());
  // map.data.insert(map.data.begin(),ary.begin(),ary.end());
  // ========== translate to OccypancyGrid type end ==========
  // MAP_PUB.publish(map);
  // printf("map size of map = %ld\n",map.data.size());
  // printf("ary size of map = %ld\n",ary.size());
}
void Img::load_map()
{
  std::string ImgPath = ros::package::getPath("localization");
  Mat img = imread(ImgPath+"/Ground.png", 0);
  for(int i=0;i<img.rows;i++)
    for(int j=0;j<img.cols;j++){
      if(img.at<uchar>(i,j)==255){
        img.at<uchar>(i,j)=0;
        // printf("1");
      }
      else{
        img.at<uchar>(i,j)=100;
      }
      if(i<72.5||i>477.5||j<72.5||j>677.5)
        img.at<uchar>(i,j)=-1;  //clear bound area 
    }
  if(img.isContinuous())
    ary.assign(img.datastart, img.dataend);
  //  imshow("likehood_map",img);
  //  waitKey(100);
}