//ros inclue
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "FIRA_status_plugin/RobotSpeedMsg.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"


//normal include
#include "strategy_nodeHandle.h"
#include "math.h"
#include "../common/Env.h"
#include <boost/algorithm/string.hpp>       //for lowercase
#include "FIRA_behavior.h"
#include "FIRA_pathplan.h"



static void show_usage()
{
    std::cerr << "Options:\n"
              << "\t-h   ,--help\t\tShow this help message\n"
              << "\t-opt,Opponent\n"
              << "\t-r  ,robot index"
              << std::endl;
}

//parse argument
//-opt,--opponent   :   set oppoent
//-r,--robotIndex   :   set robot Index
//-h,--help         :   help

void parseArg(int argc,char** argv,bool &isOpponent,int &robotIndex){

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-h") || (arg == "--help")) {
            show_usage();
            return ;
        } else if ((arg == "-r") || (arg == "--robotIndex")) {
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                robotIndex = atoi(argv[++i]); // Increment 'i' so we don't get the argument as the next argv[i].
                std::cout <<  "robotIndex=" << robotIndex << std::endl;
            } else { // Uh-oh, there was no argument to the destination option.
                  std::cerr << "--destination option requires one argument." << std::endl;
                return ;
            }
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

                std::cout <<  "strategy opponent=" << isOpponent << std::endl;
            } else { // Uh-oh, there was no argument to the destination option.
                  std::cerr << "--destination option requires one argument." << std::endl;
                return ;
            }
        }
    }

}





int main(int argc, char **argv)
{
    //=========parse arg=============//
    bool isOpponent = false;
    int shoot_value = 0;

    int robotIndex = 1;
    parseArg(argc,argv,isOpponent,robotIndex);

    isOpponent = false;

    //=========Environment init=============//
    Environment *global_env = new Environment;


    global_env->blue.pos.x = 3.35;
    global_env->blue.pos.y = 0;
    global_env->blue.pos.z = 0;

    global_env->yellow.pos.x = -3.35;
    global_env->yellow.pos.y = 0;
    global_env->yellow.pos.z = 0;

    for(int i = 0;i < PLAYERS_PER_SIDE;i++){
        global_env->home[i].v_x = 0;
        global_env->home[i].v_y = 0;
        global_env->home[i].v_yaw = 0;
        global_env->opponent[i].v_x = 0;
        global_env->opponent[i].v_y = 0;
        global_env->opponent[i].v_yaw = 0;
    }

    //=========Node Init=============//

    Strategy_nodeHandle mNodeHandle(argc,argv);
    mNodeHandle.setEnv(global_env);
    mNodeHandle.setOpponent(isOpponent);
    mNodeHandle.on_init();

    ros::Rate loop_rate(50);

    FIRA_behavior_class mbehavior;

    FIRA_pathplan_class mpathplan;

    //loadParam
    mpathplan.loadParam(mNodeHandle.getNodeHandle());
    int Team_color;
    std::cout << "=======Strategy====170119===" << std::endl;

    mbehavior.setOpponent(isOpponent);
    mpathplan.setOpponent(isOpponent);

    int *roleAry;
    int *actionAry;


    //return 0;
    while(ros::ok())
    {
        mpathplan.loadParam(mNodeHandle.getNodeHandle());
        mbehavior.loadParam(mNodeHandle.getNodeHandle());
        global_env->teamcolor = mNodeHandle.getTeamColor();
        if(global_env->teamcolor == "Blue")Team_color = Team_Blue;
        else if(global_env->teamcolor == "Yellow")Team_color = Team_Yellow;
        global_env->gameState = mNodeHandle.getGameState();
        global_env->issimulator=mNodeHandle.getIsSimulator();
        // global_env->issimulator=1;
        //printf("global_env->issimulator=%d\n",global_env->issimulator);
        shoot_value = mpathplan.getShoot();
        if(shoot_value>0){
           mNodeHandle.pubShoot(shoot_value);
           mpathplan.shoot_init();
        }
        roleAry = mNodeHandle.getRoleAry();

        //============Strategy============//
        mbehavior.setEnv(*global_env);
        mbehavior.setTeam(Team_color);
        if((global_env->issimulator)==true){
            mNodeHandle.loadParam(mNodeHandle.getNodeHandle());
            for(int i=0; i<PLAYERS_PER_SIDE;i++){
                mbehavior.readroleAry(i,roleAry[i]);
            }
            actionAry = mbehavior.getactionAry();
            mpathplan.setEnv(*global_env);
            mpathplan.setTeam(Team_color);
            for(int i=0; i<PLAYERS_PER_SIDE;i++){
                mpathplan.personalStrategy(i,actionAry[i]);
            }
        }else if((global_env->issimulator)==false){
            mNodeHandle.loadParam(mNodeHandle.getNodeHandle());
            mbehavior.readroleAry(global_env->RobotNumber,roleAry[global_env->RobotNumber]);//robotIndex is not equal role
            actionAry = mbehavior.getactionAry();
            mpathplan.setEnv(*global_env);
            mpathplan.setTeam(Team_color);
            mpathplan.personalStrategy(global_env->RobotNumber,actionAry[global_env->RobotNumber]);
        }

        //===set env===//
        Environment* tEnv = mpathplan.getEnv();

        if((global_env->issimulator)==true){
            for(int i = 0;i < PLAYERS_PER_SIDE;i++){
                    global_env->home[i].v_x = tEnv->home[i].v_x;
                    global_env->home[i].v_y = tEnv->home[i].v_y;
                    global_env->home[i].v_yaw = tEnv->home[i].v_yaw*deg2rad;
            }
        }
        else if((global_env->issimulator)==false){
            global_env->home[global_env->RobotNumber].v_x = tEnv->home[global_env->RobotNumber].v_x;
            global_env->home[global_env->RobotNumber].v_y = tEnv->home[global_env->RobotNumber].v_y;
            global_env->home[global_env->RobotNumber].v_yaw = tEnv->home[global_env->RobotNumber].v_yaw*deg2rad;
        }


        

        if(actionAry[global_env->RobotNumber] == 0){

        }else{
            mNodeHandle.pubGrpSpeed();
        }

        ros::spinOnce();
        // loop_rate.sleep();

    }

    ros::shutdown();


    std::cout << "=======Finish===" << std::endl;
    return 0;
}
