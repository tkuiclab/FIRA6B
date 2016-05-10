
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
        //std::cout << "1" << std::endl;
        global_env->home[i].v_x = 0;
        //  std::cout << "2" << std::endl;
        global_env->home[i].v_y = 0;
        global_env->home[i].v_yaw = 0;
        global_env->opponent[i].v_x = 0;
        global_env->opponent[i].v_y = 0;
        global_env->opponent[i].v_yaw = 0;
    }

//    global_env->gameState = GameState_Play;


    //=========Node Init=============//

    Strategy_nodeHandle mNodeHandle(argc,argv);
    mNodeHandle.setEnv(global_env);
    mNodeHandle.setOpponent(isOpponent);
    mNodeHandle.on_init();



    ros::Rate loop_rate(50);



    //bool isOpponent  = false;
    //FIRA_teamStrategy_class mteam;
    //mteam.setOpponent(isOpponent);
    FIRA_behavior_class mbehavior;

    FIRA_pathplan_class mpathplan;
//    mpathplan.teamColor = "Blue";
    //loadParam
    mpathplan.loadParam(mNodeHandle.getNodeHandle());
    int Team_color;
    std::cout << "=======Strategy====140811_0755===" << std::endl;
    if(mpathplan.teamColor == "Blue") Team_color = Team_Blue;
    else if(mpathplan.teamColor == "Yellow") Team_color = Team_Yellow;

//    std::cout << "[strategy] teamColor = " << mpathplan.teamColor <<  std::endl;


    mbehavior.setOpponent(isOpponent);
    mpathplan.setOpponent(isOpponent);

    int *roleAry;
    int *actionAry;


    //return 0;
    while(ros::ok())
    {
        mpathplan.teamColor = mNodeHandle.getTeamColor();
        global_env->gameState = mNodeHandle.getGameState();

        std::cout<<"ball.angle="<<global_env->home[0].ball.angle<<std::endl;
        std::cout<<"ball.distance="<<global_env->home[0].ball.distance<<std::endl;
        std::cout<<"yellow.angle="<<global_env->home[0].op_goal.angle<<std::endl;
        std::cout<<"yellow.distance="<<global_env->home[0].op_goal.distance<<std::endl;
        std::cout<<"blue.angle="<<global_env->home[0].goal.angle<<std::endl;
        std::cout<<"blue.angle="<<global_env->home[0].goal.distance<<std::endl;
        // std::cout << "[strategy] teamColor = " << mpathplan.teamColor  <<  std::endl;
        // std::cout << "[strategy] gameState = " << global_env->gameState <<  std::endl;
        roleAry = mNodeHandle.getRoleAry();
        roleAry[0] = Role_Attack;
//        printf("roleAry = %d,%d,%d\n",roleAry[0],roleAry[1],roleAry[2]);


        //start=========Strategy=============//
        mbehavior.setEnv(*global_env);
        mbehavior.setTeam(Team_color);
        for(int i=0; i<PLAYERS_PER_SIDE;i++){
            mbehavior.readroleAry(i,roleAry[i]);
        }
        actionAry = mbehavior.getactionAry();

        mpathplan.setEnv(*global_env);
        mpathplan.setTeam(Team_color);

        for(int i=0; i<PLAYERS_PER_SIDE;i++){
            mpathplan.personalStrategy(i,actionAry[i]);
        }
//        printf("%d\n",Team_color);
//printf("%d,%d,%d\n",actionAry[0],actionAry[1],actionAry[2]);



        //===set env===//
        Environment* tEnv = mpathplan.getEnv();

        for(int i = 0;i < PLAYERS_PER_SIDE;i++){

//            if(!isOpponent){
                global_env->home[i].v_x = tEnv->home[i].v_x;
                global_env->home[i].v_y = tEnv->home[i].v_y;
                global_env->home[i].v_yaw = tEnv->home[i].v_yaw*deg2rad;
//            }else{
//                global_env->opponent[i].v_x = tEnv->home[i].v_x;
//                global_env->opponent[i].v_y = tEnv->home[i].v_y;
//                global_env->opponent[i].v_yaw = tEnv->home[i].v_yaw*deg2rad;
//            }

//            double t_x=tEnv->home[i].v_x;
//            double t_y=tEnv->home[i].v_y;
//            double t_yaw=tEnv->home[i].v_yaw;
            //printf("v_x=%lf,v_y=%lf,v_yaw=%lf\n",t_x,t_y,t_yaw);
        }


        mNodeHandle.pubGrpSpeed();

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();


    std::cout << "=======Finish===" << std::endl;

    return 0;
}
