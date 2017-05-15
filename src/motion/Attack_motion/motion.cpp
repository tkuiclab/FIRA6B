/********************************
 *	Include system
 ********************************/
#include <iostream>
#include <cmath>
/********************************
 *	Include libraries
 ********************************/
#include "../common/motor_data.h"

/********************************
 *	Include header files
 ********************************/
#include "motion_nodeHandle.h"
#include "base_control.h"
//#include "CTest.h"
/********************************
 *	Define	
 ********************************/
//#define DEBUG

int main(int argc, char **argv)
{
	Motion_nodeHandle Global_nodeHandle(argc, argv);
	Base_Control Global_Base_Control;

	motor_command *CMD = new motor_command;
	//while(ros::ok()){
	//	if(Global_Motor_Control.mcssl_init()){
	//		break;
	//	}else{
	//		exit(EXIT_FAILURE);
	//	}
	//}
	ros::Rate loop_rate(30);
	while(ros::ok()){
		CMD = Global_nodeHandle.getMotion();
		Global_Base_Control.send(CMD);	
		if(CMD->shoot_power>0)Global_nodeHandle.clearshoot();
		ros::spinOnce();
		loop_rate.sleep();
	}
	delete CMD;
//	ros::shutdown();
	std::cout << "close Attack Motion\n";
#ifdef DEBUG
#else
#endif
	return 0;
}


