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
	double yaw_speed;
	myLocation_x = env->robot.pos.x;
	myLocation_y = env->robot.pos.y;
	myLocation_yaw = env->robot.pos.z*M_PI/180;
    env->robot.v_x = (-15)*(point_y-myLocation_y);
    env->robot.v_y = (15)*(point_x-myLocation_x);
	//printf("%f\t%f\t%f\n",point_x-myLocation_x, point_y-myLocation_y, env->robot.pos.z);
	//printf("%f\t%f\n",cos(myLocation_yaw), sin(myLocation_yaw));

	//env->robot.v_x = (-10)*(point_y-myLocation_y);
	//env->robot.v_y = (10)*(point_x-myLocation_x);
	yaw_speed = (-1)*(env->robot.pos.z+90);
	env->robot.v_yaw = (fabs(yaw_speed)>10)? yaw_speed/fabs(yaw_speed)*10: yaw_speed;
}

void PathPlan::chaseBall(Environment *env)
{
	double ball_dis, ball_angle;
	double yaw_speed;
	ball_dis = env->robot.ball.distance;
	ball_angle = env->robot.ball.angle;
	//env->robot.v_x = (10)*ball_dis*sin(ball_angle*M_PI/180);
	if(fabs(ball_angle)>3){
		env->robot.v_x = (-10)*ball_dis*sin(ball_angle*M_PI/180);
		env->robot.v_y = 0;
		env->robot.v_yaw = (fabs(yaw_speed)>5)? yaw_speed/fabs(yaw_speed)*5: yaw_speed;	
		//env->robot.v_yaw = (-1) * (env->robot.pos.z+90);
	}else{

		env->robot.v_x = 0;
		env->robot.v_y = (10)*ball_dis*cos(ball_angle*M_PI/180);
		yaw_speed = ball_angle;
		env->robot.v_yaw = (fabs(yaw_speed)>5)? yaw_speed/fabs(yaw_speed)*5: yaw_speed;	
	}
}

void PathPlan::aimTargetCone(Environment *env, double goal_angle)
{
	double yaw_speed;
	env->robot.v_x = 0;
	env->robot.v_y = 0;
	yaw_speed = (-1)*goal_angle;
	env->robot.v_yaw = (fabs(yaw_speed)>5)? yaw_speed/fabs(yaw_speed)*5: yaw_speed;
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
