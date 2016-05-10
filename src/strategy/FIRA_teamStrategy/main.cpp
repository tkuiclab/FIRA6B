//ros inclue
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "FIRA_status_plugin/RobotSpeedMsg.h"


//our file
#include "FIRA_teamStrategy.h"
#include "teamStrategy_nodeHandle.h"
#include"../common/Env.h"
#include "math.h"

//other include
//#include <unistd.h>
#include <boost/algorithm/string.hpp>       //for lowercase

//#define Pub_Speed_Topic "/FIRA/Strategy/PathPlan/RobotSpeed"

static void show_usage()
{
    std::cerr << "Options:\n"
    << "\t-h,--help\t\tShow this help message\n"
    << "\t-r,robot index"
    << std::endl;
}




//parse argument
//-opt,--opponent   :   set oppent
//-h,--help         :
void parseArg(int argc,char** argv,bool &isOpponent){
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-h") || (arg == "--help")) {
            show_usage();
            return ;
            /*
             } else if ((arg == "-r") || (arg == "--robotIndex")) {
             if (i + 1 < argc) { // Make sure we aren't at the end of argv!
             robotIndex = atoi(argv[++i]); // Increment 'i' so we don't get the argument as the next argv[i].
             std::cout <<  "robotIndex=" << robotIndex << std::endl;
             } else { // Uh-oh, there was no argument to the destination option.
             std::cerr << "--destination option requires one argument." << std::endl;
             return ;
             }
             }*/
        } else if ((arg == "-opt") || (arg == "--opponent")) {
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                //robotIndex = atoi(argv[++i]); // Increment 'i' so we don't get the argument as the next argv[i].
                std::string nextArg = argv[++i];
                boost::algorithm::to_lower(nextArg);
                if(nextArg=="true"){
                    isOpponent = true;
                }else{
                    isOpponent = false;
                }
                
                std::cout <<  "teamStrategy opponent=" << isOpponent << std::endl;
            } else { // Uh-oh, there was no argument to the destination option.
                std::cerr << "--destination option requires one argument." << std::endl;
                return ;
            }
        }
    }
    
}


int main(int argc, char **argv)
{
    
    std::cout << "=======FIRA Team Strategy Start====140810_0755===" << std::endl;
    
    //---parse argument---
    bool isOpponent = false;     //---------------------------------------改Opponent
    parseArg(argc,argv,isOpponent);
    
    Environment *global_env = new Environment;
    //goal input
    global_env->blue.pos.x = 3.35;
    global_env->blue.pos.y = 0;
    global_env->blue.pos.z = 0;
    
    global_env->yellow.pos.x = -3.35;
    global_env->yellow.pos.y = 0;
    global_env->yellow.pos.z = 0;
    
    
    
    //---node init---
    TeamStrategy_nodeHandle mNodeHandle(argc,argv);
    mNodeHandle.setEnv(global_env);
    mNodeHandle.setOpponent(isOpponent);
    mNodeHandle.on_init();
    
    //std::cout <<  "teamStrategy (in main) opponent=" << isOpponent << std::endl;
    
    ros::Rate loop_rate(10000);
    
    //global_env->gameState = GameState_AvoidBarrier;
    
    //teamStrategy init
    FIRA_teamStrategy_class mteam;
    
    mteam.setOpponent(isOpponent);
    mteam.loadParam(mNodeHandle.getNodeHandle());
    
    
    
    
    
    int *roleAry;
    while(ros::ok())
    {
        //std::cout << "before team of A" << std::endl;
        
        mteam.setEnv(*global_env);
        mteam.teamStrategy();
        roleAry = mteam.getRoleAry();
        
        mNodeHandle.pubRole(roleAry);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    ros::shutdown();
    
    
    std::cout << "=======FIRA Team Strategy <<Finish>>===" << std::endl;
    
    return 0;
}