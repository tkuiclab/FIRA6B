/********************************
 *	Include system
 ********************************/
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
/********************************
 *	Include libraries
 ********************************/
#include "motor_data.h"

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
	Motion_nodeHandle main_nodeHandle(argc, argv);
	Base_Control main_Base_Control;

	robot_command *main_robotCMD;
	//while(ros::ok()){
	//	if(Global_Motor_Control.mcssl_init()){
	//		break;
	//	}else{
	//		exit(EXIT_FAILURE);
	//	}
	//}
	ros::Rate loop_rate(30);
	while(ros::ok()){
		main_robotCMD = main_nodeHandle.getMotion();
		main_Base_Control.send(main_robotCMD);
		
		main_nodeHandle.clear();

		ros::spinOnce();
		loop_rate.sleep();
	}
	delete main_robotCMD;
//	ros::shutdown();
	std::cout << "close Attack Motion\n";
#ifdef DEBUG
#else
#endif
	return 0;
}


