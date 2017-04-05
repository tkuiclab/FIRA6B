#include "ros/ros.h"
#include "param_convey/strategy_param.h"

bool dump(param_convey::strategy_param::Request  &req,
         param_convey::strategy_param::Response &res)
{
  if(req.receive == 1){
    req.receive = 0;
    ROS_INFO("Updating infos...");
    system("rosparam dump ~/FIRA17_ws/src/fira_launch/default_config/vision_better.yaml");
    ROS_INFO("Update Success!");
    res.update = 2;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "strategy_param_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("StrategyParam", dump);
  ROS_INFO("Ready to convey the strategy's parameters.");
  ros::spin();

  return 0;
}
