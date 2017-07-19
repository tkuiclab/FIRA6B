#ifndef pathplan_H
#define pathplan_H

class PathPlan{
	PathPlan();
	~PathPlan();
private:
	void level1();
	void level2();
	void level3();
	void level4();
public:
	void setLevel();
};
#endif
