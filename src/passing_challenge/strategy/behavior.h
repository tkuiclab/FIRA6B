#ifndef behavior_H
#define behavior_H
/********************************
 *			Include				*
 ********************************/
#include "strategy_nodeHandle.h"
#include "pathplan.h"
/********************************
 *			Define				*
 ********************************/
#define Level1 1
#define Level2 2
#define Level3 3
#define Level4 4
#define Target1 1
#define Target2 2
#define Target3 3
#define Target4 4
class Behavior : public Strategy_nodeHandle , private PathPlan{
public:
	Behavior(int, char**);
	~Behavior();
private:
	int gameStatus;
	int target;
	Environment *env;
	Ball ball_1, ball_2, ball_3, ball_4;
	Goal goal_1, goal_2, goal_3, goal_4;
	//Strategy_nodeHandle *nn;
private:
	void chooseLevel();
	int setStatus();
	void start();
	void halt();
	void chase(const Ball);
	void aim(const Goal);
	void shoot();	
	void run();
	void finish();
	bool holdBall();
	void nextTarget();
	Ball getTargetBall();
	Goal getTargetGoal();
public:
	int getGameState(){return gameStatus;}
};
#endif
