#ifndef pathplan_H
#define pathplan_H

#include "Env.h"
class PathPlan{
public:
	PathPlan();
	~PathPlan();
public:
	void gotoPoint(Environment*, double, double, double);
	void chaseBall(Environment*);
	void aimTargetCone(Environment*, double);
private:
	void transform(Environment*, const double &, const double &); 
};
#endif
