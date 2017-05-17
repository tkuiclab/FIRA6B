cmake_minimum_required(VERSION 2.8.3)
project(motion)

find_package(catkin REQUIRED COMPONENTS
 qt_build
 message_generation
 std_msgs
 roscpp
)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS
 message_runtime
 std_msgs
 roscpp
)

#foreach(subdir
#		common
#		Attack_motion
#		)
#	add_subdirectory(${subdir})
#endforeach()
add_library(csslLib cssl/cssl.h cssl/cssl.c cssl/uty.c)

add_executable(motion motion.cpp MotorControl.h)
target_link_libraries(motion ${catkin_LIBRARIES})
target_link_libraries(motion csslLib)

