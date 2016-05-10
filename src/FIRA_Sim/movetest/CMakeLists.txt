##############################################################################
# Sections
##############################################################################

set(QT_FORMS ${CMAKE_CURRENT_SOURCE_DIR}/../ui/FIRA_Sim_window.ui
             ${CMAKE_CURRENT_SOURCE_DIR}/../ui/image_widget.ui
)
set(QT_RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../resources/images.qrc)


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

add_executable(movetest ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_dependencies(movetest re_status_plugin_gencpp)
target_link_libraries(movetest FIRA_Sim PF_Lib)
install(TARGETS movetest RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

