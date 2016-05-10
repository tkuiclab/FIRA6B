##############################################################################
# Sections
##############################################################################

##############################################################################
# Sources
##############################################################################


##############################################################################
# Binaries
##############################################################################

add_library(BaseNodeLib BaseNode.cpp Env.h)
target_link_libraries(BaseNodeLib ${catkin_LIBRARIES})
#install(TARGETS BaseNodeLib
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
#)
