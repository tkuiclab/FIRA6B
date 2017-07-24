#include "pathplan.h"
PathPlan::PathPlan()
{

}

PathPlan::~PathPlan()
{

}

void PathPlan::gotoPoint(Environment *env, double point_x, double point_y)
{
	double myLocation_x, myLocation_y, myLocation_yaw, yaw_degree;
	myLocation_x = env->robot.pos.x;
	myLocation_y = env->robot.pos.y;
	myLocation_yaw = env->robot.pos.z*M_PI/180;
	env->robot.v_x = (point_x-myLocation_x)*cos(myLocation_yaw+(M_PI/2)) + (point_y-myLocation_y)*sin(myLocation_yaw+(M_PI/2));
	env->robot.v_y = (point_y-myLocation_y)*cos(myLocation_yaw+(M_PI/2)) + (point_x-myLocation_x)*sin(myLocation_yaw+(M_PI/2));
	env->robot.v_yaw = -(myLocation_yaw+(M_PI/2));
}

void PathPlan::chaseBall(Environment *env)
{
	double ball_dis, ball_angle;
	ball_dis = env->robot.ball.distance;
	ball_angle = env->robot.ball.angle;
	env->robot.v_x = ball_dis*cos(ball_angle*M_PI/180);
	env->robot.v_y = ball_dis*sin(ball_angle*M_PI/180);
	env->robot.v_yaw = ball_angle;
}

void PathPlan::aimTargetCone(Environment *env, double goal_angle)
{
	env->robot.v_x = 0;
	env->robot.v_y = 0;
	env->robot.v_yaw = goal_angle;
}

void PathPlan::level1()
{     
      
}     
      
void PathPlan::level2()
{     
      
}     
      
void PathPlan::level3()
{     
      
}     
      
void PathPlan::level4()
{     
      
}     
      
void PathPlan::setLevel()
{

}
