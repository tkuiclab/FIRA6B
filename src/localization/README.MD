
# User instruction #

## open the motion, camera and static tf node ##
roslaunch localization localization.launch
## open the odom and transfer white_line data into scan ##
rosrun localization localization_node
## open the imu ##
sudo ./devel/lib/imu_3d/imu_3d
## open the ekf package and the amcl node ##
roslaunch localization amcl.launch
## open the akf node ##
rosrun localization AKF_node
## open the reset amcl node ##
rosrun localization reset_controller