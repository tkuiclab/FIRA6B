
#ifndef _ENV_H_
#define _ENV_H_
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;

const long PLAYERS_PER_SIDE = 3;

#define GameState_Halt          0
#define GameState_Play          1
#define GameState_FreeKick      2
#define GameState_PenaltyKick   3
#define GameState_FreeBall      4
#define GameState_ThrowIn       5
#define GameState_CornerKick    6
#define GameState_GoalKick      7
#define GameState_AvoidBarrier  8

//#define goalkeeper              8
//#define attack                  9
//#define support                 10

#define Role_Halt               0
#define Role_Goalkeeper         1
#define Role_Attack             2
#define Role_Support            3
#define Role_PenaltyKick        4
#define Role_ThrowIn            5
#define Role_CornerKick         6
#define Role_AvoidBarrier       7

#define action_Halt             0
#define action_Goalkeeper       1
#define action_Attack           2
#define action_Support          3
#define action_AvoidBarrier     4
#define action_typeS_attack     5
#define action_Chase            6
#define action_typeU_Attack     7
#define action_SideSpeedUp      8
#define action_CornerKick       9
#define action_Zone_Attack     10
#define action_1               11
#define action_2               12
#define action_3               13
#define action_4               14
#define action_5               15
#define action_6               16
#define action_Dorsad_Attack   17
#define action_Shoot_Attack    18
#define action_Straight_Chase  19

#define state_Init              0
#define state_Chase             1
#define state_Attack            2
#define state_UChase            3
#define state_SAttack           4
#define state_SideSpeedUp       5
#define state_ZoneAttack        6
#define state_CornerKick        7

#define Team_Blue 1
#define Team_Yellow 2


#define Goal_Blue 1
#define Goal_Yellow 2


typedef struct{
    double x,y,z;
    double distance, angle;
}Vector3D;

typedef struct{
    Vector3D pos;
    Vector3D ball;
    Vector3D goal;
    Vector3D op_goal;
    double rotation;
    double v_x,v_y,v_yaw;
    //double velocityLeft, velocityRight;
}Robot;


typedef struct{
    Vector3D pos;
}Ball;
typedef struct{
    Vector3D pos;
}Goal;
typedef struct
{
    //nodeHandle
    int Blackangle;
    int RobotNumber;
    int IsSimulator;
    std::vector<double> SPlanning_Velocity;
    std::vector<double> Distance_Settings;
    //behavior
    std::vector<double> Bahavior_Attack_Strategy;
    std::vector<double> Bahavior_Chase_Strategy;
    std::vector<double> Bahavior_Corner_Kick;
    std::vector<double> Bahavior_Side_Speed_UP;
    std::vector<double> Bahavior_TypeS_Attack;
    std::vector<double> Bahavior_TypeU_Chase;
    std::vector<double> Bahavior_Zone_Attack;
    std::vector<double> Bahavior_Dorsad_Attack;
    std::vector<int> Bahavior_StrategySelection;
    //pathplan
    std::vector<double> Pathplan_Attack_Strategy;
    std::vector<double> Pathplan_Chase_Strategy;
    std::vector<double> Pathplan_Zone_Attack;
    std::vector<double> Pathplan_TypeS_Attack;
    std::vector<double> Pathplan_TypeU_Attack;
    std::vector<double> Pathplan_Dorsad_Attack;
    std::vector<double> Pathplan_Corner_Kick;
    std::vector<double> Pathplan_SideSpeedUp;
} Parameter;

typedef struct
{
    Robot home[PLAYERS_PER_SIDE];
    Robot opponent[PLAYERS_PER_SIDE];
    Ball currentBall, lastBall, predictedBall;
    //Bounds fieldBounds, goalBounds;
    long gameState;
    int RobotNumber;
    int  issimulator;
    std::string teamcolor;
    //BlackObject
    int blackangle[20];
    int mindis[20];
    //long whosBall;
    //void *userData;
    Goal yellow, blue;
    int SaveParam;
    Parameter param;
} Environment;



//static Environment global_env;





#define pi 3.14159
#define very_small 0.000001

#define rad2deg 180/pi
#define deg2rad pi/180

#define half_robot 0.12

#define SIGN(A) ( (A)>=0?1:-1)


#endif
