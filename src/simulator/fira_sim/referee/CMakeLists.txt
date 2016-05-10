##############################################################################
# Sections
##############################################################################

set(QT_FORMS ${CMAKE_CURRENT_SOURCE_DIR}/ui/referee_window.ui
             ${CMAKE_CURRENT_SOURCE_DIR}/ui/connect_ros_window.ui)
set(QT_RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/resources/images.qrc)


file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})


##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(referee ${QT_FORMS_HPP} ${QT_SOURCES} ${QT_RESOURCES_CPP}  ${QT_MOC_HPP})
target_link_libraries(referee qnodeLib)
install(TARGETS referee RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#install(TARGETS model RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#message(CATKIN_PACKAGE_BIN_DESTINATION=${CATKIN_PACKAGE_BIN_DESTINATION})
