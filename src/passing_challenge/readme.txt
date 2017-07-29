roslaunch passingChallenge passingChallenge.launch
rosrun localization localization_node
sudo ./devel/lib/imu_3d/imu_3d
roslaunch passingChallenge amcl.launch
rosrun passingChallenge PassingChallenge_node
