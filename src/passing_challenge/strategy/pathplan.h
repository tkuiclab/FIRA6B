#ifndef pathplan_H
#define pathplan_H

#include "Env.h"
class PathPlan{
public:
	PathPlan();
	~PathPlan();
public:
	void gotoPoint(Environment*, double, double);
	void chaseBall(Environment*);
	void aimTargetCone(Environment*, double);
	void level1();
	void level2();
	void level3();
	void level4();
public:
	void setLevel();
};
#endif
