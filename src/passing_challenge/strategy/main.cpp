#include <iostream>
//#include "strategy_nodeHandle.h"
#include "behavior.h"
//#include "pathplan.h"

int main(int argc, char** argv)
{
	//Strategy_nodeHandle main_nodeHandle(argc, argv);
	Behavior main_behavior(argc, argv);
	//PathPlan main_pathplan();
//	ros::Rate loop_rate(50);
	/*while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}*/
	std::cout << "Close Passing Challenge Strategy\n";
	return 0;
}
