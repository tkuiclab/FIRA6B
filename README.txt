===============   How to perform the localization program   ===============
source devel/setup.bash
roslaunch localization localization.launch
///////////////   Open the new terminal   ///////////////////////
source devel/setup.bash
rosrun localization localization_node
===============   How to save the map   ===============
rosrun map_sever map_saver -f fira_map
