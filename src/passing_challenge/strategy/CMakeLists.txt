
##############################################################################
# Sections
##############################################################################


##############################################################################
# Sources
##############################################################################
add_definitions(-std=c++11)

set(INCLUDE_DIR
	../common
)
include_directories(${INCLUDE_DIR})

##############################################################################
# Binaries
##############################################################################


add_executable(PassingChallenge strategy_nodeHandle.cpp main.cpp behavior.cpp pathplan.cpp)
target_link_libraries(PassingChallenge PassingChellengeLib ${catkin_LIBRARIES})
