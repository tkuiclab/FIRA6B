/**
 * @file /eros_qtalker/src/BaseNode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include "BaseNode.h"
#include <std_msgs/String.h>
#include <sstream>

/*****************************************************************************
** Implementation
*****************************************************************************/

BaseNode::BaseNode(int argc, char** argv, const std::string &name ) :
	init_argc(argc),
	init_argv(argv),
	node_name(name)
	{}

BaseNode::~BaseNode() {
	shutdown();
}
/**
 * This is called by the qt application to stop the ros node before the
 * qt app closes.
 */
void BaseNode::shutdown() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool BaseNode::on_init() {

    std::cout << "node_name=" << node_name << std::endl;
	ros::init(init_argc,init_argv,node_name);
    std::cout << "BaseNode::on_init() say after init" << std::endl;

    if ( ! ros::master::check() ) {
        std::cout << "BaseNode::on_init() say Node Check false" << std::endl;

        return false;
	}

    std::cout << "BaseNode::on_init() say  afetr check " << std::endl;

	ros::start(); // our node handles go out of scope, so we want to control shutdown explicitly.

    std::cout << "BaseNode::on_init() say  afetr start " << std::endl;

    ros_comms_init();

	return true;
}

bool BaseNode::on_init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,node_name);
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // our node handles go out of scope, so we want to control shutdown explicitly.
	ros_comms_init();

	return true;
}

