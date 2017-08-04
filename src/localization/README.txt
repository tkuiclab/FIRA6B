/**
 *
 *  @ file  User manual for AMCL with robot_pose_ekf 
 *  
 *  @ brief Introduce how to use localization package
 *
 *  @ date August 2017
 **/

  
// *****************   open the motion , camera and static_tf   *********************//
 
    roslaunch localization localization.launch

// *****************   open the odom and transfer white_line data into scan   *********************//

    rosrun localization localization_node

// *****************   open the imu    *********************//

    sudo ./devel/lib/imu_3d/imu_3d

// *****************   open the ekf package and the amcl node   *********************//

    roslaunch localization amcl.launch

// *****************   end   *********************//