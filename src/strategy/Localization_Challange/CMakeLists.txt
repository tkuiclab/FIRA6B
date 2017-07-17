
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

add_executable(LocalizationChallange main.cpp NodeHandle.cpp Strategy.cpp)
target_link_libraries(LocalizationChallange ${catkin_LIBRARIES} BaseNodeLib)
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
