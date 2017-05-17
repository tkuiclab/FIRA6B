cmake_minimum_required(VERSION 2.8.7 FATAL_ERROR)
project(FIRA_status_plugin)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall")


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
##add_service_files(DIRECTORY srv FILES ModelCommand.srv)
add_message_files(FILES
                    ModelMsg.msg
                    RobotSpeedMsg.msg
)

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

#add_library(soccer_plugin SHARED SoccerPlugin.cc)
#add_dependencies(soccer_plugin FIRA_status_plugin_gencpp)
#add_dependencies(soccer_plugin ${catkin_EXPORTED_TARGETS})
#target_link_libraries(soccer_plugin  ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})


add_library(FIRA_world_plugin SHARED FIRA_world_plugin.cc)
add_dependencies(FIRA_world_plugin fira_sim_gencpp)
target_link_libraries(FIRA_world_plugin
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROTUBUF_LIBRARIES}
)


#add_library(contact SHARED ContactPlugin.cc)
#target_link_libraries(contact ${GAZEBO_libraries})

#add_dependencies(re_status re_status_msgs_generate_messages_cpp)

#add_executable (test test.cc)
#target_link_libraries(test model_push_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
#add_dependencies(test model_push_msgs)
