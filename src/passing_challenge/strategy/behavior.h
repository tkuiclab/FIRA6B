#ifndef behavior_H
#define behavior_H
/****************************
 *		Include			    *
 ****************************/
#include "strategy_nodeHandle.h"
#include "pathplan.h"
class Behavior : private Strategy_nodeHandle{
public:
	Behavior(int, char**);
	~Behavior();
private:
	int gameStatus;
	//Strategy_nodeHandle *nn;
private:
	void chooseLevel();
	void setStatus();
	void start();
	void halt();
	void chase();
	void aim();
public:
	int getGameState(){return gameStatus;}
};
#endif
