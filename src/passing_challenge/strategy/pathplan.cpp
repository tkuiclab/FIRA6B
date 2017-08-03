#include "pathplan.h"
PathPlan::PathPlan()
{

}

PathPlan::~PathPlan()
{

}

void PathPlan::gotoPoint(Environment *env, double point_x, double point_y, double angle)
{
	double myLocation_x, myLocation_y, myLocation_yaw, yaw_degree;
	double x_speed, y_speed, yaw_speed;
	myLocation_x = env->robot.pos.x;
	myLocation_y = env->robot.pos.y;
	myLocation_yaw = env->robot.pos.z*M_PI/180;

    x_speed = (15)*((point_x-myLocation_x)*cos(myLocation_yaw) + (point_y-myLocation_y)*sin(myLocation_yaw));
    y_speed = (15)*((-1)*(point_x-myLocation_x)*sin(myLocation_yaw) + (point_y-myLocation_y)*cos(myLocation_yaw));

	yaw_speed = (fabs(angle)>180)? (-1)*angle : angle;
	env->robot.v_yaw = (fabs(yaw_speed)>10)? yaw_speed/fabs(yaw_speed)*10: yaw_speed;

	transform(env, x_speed, y_speed);
}

void PathPlan::chaseBall(Environment *env)
{
	double ball_dis, ball_angle;
	double x_speed, y_speed, yaw_speed;
	ball_dis = env->robot.ball.distance;
	ball_angle = env->robot.ball.angle;
	
	x_speed = (15)*sin(ball_angle*M_PI/180);
	y_speed = (15)*cos(ball_angle*M_PI/180);
	env->robot.v_yaw = (fabs(ball_angle)>10)? ball_angle/fabs(ball_angle)*10: ball_angle;

	transform(env, x_speed, y_speed);
}

void PathPlan::aimTargetCone(Environment *env, double goal_angle)
{
	double x_speed, y_speed;
	x_speed = 0;
	y_speed = 0;
	env->robot.v_yaw = (fabs(goal_angle)>5)? goal_angle/fabs(goal_angle)*5: goal_angle;

	transform(env, x_speed, y_speed);
}

void PathPlan::transform(Environment *env, const double &v_x, const double &v_y)
{
	env->robot.v_x = (-1)*v_y;
	env->robot.v_y = v_x;
	//env->robot.v_x = v_x;
	//env->robot.v_y = v_y;
}
