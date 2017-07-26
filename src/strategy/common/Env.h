
#ifndef _ENV_H_
#define _ENV_H_

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
#define Role_Test1              8
#define Role_Test2              9
#define Role_Test3             10
#define Role_NewSupport        11
#define Role_Kick              12

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
#define action_PenaltyKick     20
#define action_ThrowIn         21
#define action_Support_CatchBallState   22
#define action_Support_LostBallState    23
#define action_Support_Test1            24
#define action_Support_Test2            25
#define action_Support_Test3            26
#define action_Support_Positioning      27
#define action_MovetoBlueGate           28
#define action_MovetoYellowGate         29
#define action_LeaveBall                30
#define action_LeaveLimitArea           31
#define action_LeftRightMove            32
#define action_inv_LeftRightMove        33
#define action_Support_LostInternet     34
#define action_MovetoGoal               35
#define action_MovetoOpGoal             36
#define action_MovetoGoalEdge1          37
#define action_MovetoGoalEdge2          38
#define action_MovetoOpGoalEdge1        39
#define action_MovetoOpGoalEdge2        40
#define action_Stop                     41
#define action_Block                    42
#define action_Kick                     43

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
    double angle_1, angle_2;
}Two_point;

typedef struct{
    Vector3D pos;
    Vector3D ball;
    Vector3D goal;
    Vector3D op_goal;
    double rotation;
    double v_x,v_y,v_yaw;
    Two_point goal_edge, op_goal_edge;
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
    int AnotherGetBall;
    double AnotherBallDistance;
    int AnotherRobotNumber;
    double AnotherGoalDistance;
    int R1OrderR2;
    double Support_Obstacle_angle;
    double Support_Obstacle_distance;
    int isteamstrategy;
} Environment;
//static Environment global_env;

typedef struct
{
    int begin;
    int end;
}Range;


#define pi 3.14159
#define very_small 0.000001

#define rad2deg 180/pi
#define deg2rad pi/180

#define half_robot 0.12

#define SIGN(A) ( (A)>=0?1:-1)
#define speed_constant 2
#define speed_limit 0.01
#define yaw_speed_limit 1

#endif
