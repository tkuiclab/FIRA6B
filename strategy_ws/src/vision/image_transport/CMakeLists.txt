##############################################################################
# Sections
##############################################################################

file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp)
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)

##############################################################################
# Binaries
##############################################################################

add_library(image_transportcpp image_transport.cpp)
add_executable(image_transport ${QT_SOURCES} ${QT_MOC_HPP})
target_link_libraries(image_transport visionLib image_transportcpp)
#install(TARGETS whiteLine RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#message(CATKIN_PACKAGE_BIN_DESTINATION=${CATKIN_PACKAGE_BIN_DESTINATION})
