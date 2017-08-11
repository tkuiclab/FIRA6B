##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(strategy)
find_package(catkin)
##using c++ 11 standard
add_definitions(-std=c++11)

##############################################################################
# Catkin Configuration
##############################################################################
#message(kkkkkTest catkin_INCLUDE_DIRS=${catkin_INCLUDE_DIRS})
# These find_package calls setup catkin_INCLUDE_DIRS and ROS_LIBRARIES and 
# all the component variables too (e.g. roscpp_INCLUDE_DIR?)
find_package(catkin REQUIRED COMPONENTS qt_build message_generation std_msgs roscpp
	     sensor_msgs cv_bridge image_transport)
include_directories(${catkin_INCLUDE_DIRS})

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in manifest.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here
add_definitions(-std=c++11)
##############################################################################
# Services
##############################################################################

add_message_files(
   FILES
   strategy.msg
 )
#add_service_files(DIRECTORY srv FILES TwoInts.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime roscpp rospy std_msgs sensor_msgs cv_bridge image_transport
)


##############################################################################
# Subprojects
##############################################################################

foreach(subdir
    common
    FIRA_strategy
    FIRA_teamStrategy
    Localization_Challange
    )
  add_subdirectory(${subdir})
endforeach()

##############################################################################
# Optional Installs - these aren't really necessary
##############################################################################
#message(kkkkkTest CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX})

#install(DIRECTORY resources DESTINATION ${CMAKE_INSTALL_PREFIX}/share/qt_tutorials)
#install(DIRECTORY ui DESTINATION ${CMAKE_INSTALL_PREFIX}/share/qt_tutorials)
