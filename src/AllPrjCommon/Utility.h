/**
 * @file Utility.h
 *
 * @brief All Utility, Tool
 *
 * @date February 2014
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef UTILITY_H_
#define UTILITY_H_

/*****************************************************************************
** Includes
*****************************************************************************/


#include <string>
#include <iostream>

/*****************************************************************************
** Class
*****************************************************************************/



class Utility  {
public:

    static void show_usage()
    {
        std::cerr << "Options:\n"
                  << "\t-h   ,--help\t\tShow this help message\n"
                  << "\tR1,R2,R3 , Robot Index\n"
                  << std::endl;
    }
  
    //parse argument,get robot index
    //input: argc, argv
    //output : robotIndex
    static int parseArg_getRobotIndex(int argc,char** argv){
        std::string robotR;
	    for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if ((arg == "R1") || (arg == "R2") || arg == "R3") {
                robotR = arg;
                char tmp = robotR.c_str()[1];

                return (tmp - 48);
            }else if ((arg == "-h") || (arg == "--help")) {
                show_usage();
                return -1;
            }
	    }
	    
        return -1;

	}
};

#endif /* NODE_HPP_ */

