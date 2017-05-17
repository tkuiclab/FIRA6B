cmake_minimum_required(VERSION 2.8.7 FATAL_ERROR)
project(re_status_plugin)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall")


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_service_files(DIRECTORY srv FILES ModelCommand.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

#catkin_package()

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  )
link_directories(${GAZEBO_LIBRARY_DIRS} ${SDF_LIBRARY_DIRS})
#add_subdirectory(srv)

add_library(re_status SHARED re_status.cc )
add_dependencies(re_status re_status_plugin_gencpp)
target_link_libraries(re_status  ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(re_status re_status_msgs)


#add_executable (test test.cc)
#target_link_libraries(test model_push_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
#add_dependencies(test model_push_msgs)
