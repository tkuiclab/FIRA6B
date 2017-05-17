cmake_minimum_required(VERSION 2.8.3)
project(Attack_motion)


add_executable(Attack_motion_node 
				motion.cpp 
				motion_nodeHandle.cpp# base_control.cpp 
				#CTest.cpp
				)
target_link_libraries(Attack_motion_node ${catkin_LIBRARIES} csslLib motorDataLib)

