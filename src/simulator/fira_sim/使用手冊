1.編譯方法
1.1.使用CMakeLists在catkin_ws下編譯
1.2.複製plugin
  #cd ~/FIRA_ws/devel/lib 
  #cp libFIRA_robot_pub_plugin.so libobj_pub_plugin.so ~/gztest/gzplugin/
1.3.複製model
  #cp -rf blue_goal/ yellow_goal/ my_robot/ fira_ground/ ~/gztest/gzmodel/	

1.4.確定youbot_ros_tools已在workspace底下
1.5.catkin_make


2.執行方法
2.1.Roslaunch fira_simulator.launch
＃roslaunch fira_sim fira_simulator.launch
2.2.執行程式
##執行策略(策略要寫在這裡)(或再QT下執行)
＃rosrun strategy FIRA_strategy

3.控制語法

3.1.reset
###rostopic pub /FIRA/Simulaotr/WorldPlugin  std_msgs/String "data: 'reset'" 
#rosrun fira_sim referee

3.2.控制單一機器人
#roslaunch fira_sim basic.launch
#roslaunch fira_sim fira_oneRobot.launch
#roslaunch youbot_teleop youbot_teleop.launch ns:=/youbot0


4.Other

4.1.refer:
https://github.com/micpalmia/youbot_ros_tools


4.2.Edit Gazebo Path
sudo vim /usr/share/gazebo/setup.sh
sudo vim /usr/share/gazebo1.9/setup.sh

4.3.Test Collision
rosrun gazebo_ros gazebo  contact.world






