##############################################################################
# CMake
##############################################################################
cmake_minimum_required(VERSION 2.8.0)
project(vision)
find_package(catkin)


#message(QT_QTUITOOLS_INCLUDE_DIR=${QT_QTUITOOLS_INCLUDE_DIR})
##############################################################################
# Catkin Configuration
##############################################################################
#message(kkkkkTest catkin_INCLUDE_DIRS=${catkin_INCLUDE_DIRS})
# These find_package calls setup catkin_INCLUDE_DIRS and ROS_LIBRARIES and 
# all the component variables too (e.g. roscpp_INCLUDE_DIR?)
#find_package(catkin REQUIRED COMPONENTS qt_build message_generation std_msgs roscpp)
find_package(catkin REQUIRED COMPONENTS qt_build message_generation std_msgs roscpp
	     sensor_msgs cv_bridge image_transport)
include_directories(${catkin_INCLUDE_DIRS})

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in manifest.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Services
##############################################################################

#add_service_files(DIRECTORY srv FILES TwoInts.srv)
#add_service_files(
#  FILES
#)
add_message_files(
  FILES
  Object.msg
)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs roscpp 
		sensor_msgs cv_bridge image_transport)

##############################################################################
# Subprojects
##############################################################################

foreach(subdir
    common
    Interface
    objectdetection
    whiteLine
    blackItem
    image_transport
)
  #install(DIRECTORY ${subdir} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/qt_tutorials)
  add_subdirectory(${subdir})
endforeach()

##############################################################################
# Optional Installs - these aren't really necessary
##############################################################################
#message(kkkkkTest CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX})
#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()


#install(DIRECTORY resources DESTINATION ${CMAKE_INSTALL_PREFIX}/share/qt_tutorials)
#install(DIRECTORY ui DESTINATION ${CMAKE_INSTALL_PREFIX}/share/qt_tutorials)

