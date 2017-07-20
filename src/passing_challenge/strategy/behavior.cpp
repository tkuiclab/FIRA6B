#include "behavior.h"

Behavior::Behavior(int argc, char** argv)
{
	//nn = new Strategy_nodeHandle();
	env = getEnv();
	init(argc, argv);
	target = 1;
	ball_1.pos.x = -165;
	ball_1.pos.y = 120;
	ball_2.pos.x = -165;
	ball_2.pos.y = 40;
	ball_3.pos.x = -165;
	ball_3.pos.y = -40;
	ball_4.pos.x = -165;
	ball_4.pos.y = -120;
	
	goal_1.pos.x = 165;
	goal_1.pos.y = 120;
	goal_2.pos.x = 165;
	goal_2.pos.y = 40;
	goal_3.pos.x = 165;
	goal_3.pos.y = -40;
	goal_4.pos.x = 165;
	goal_4.pos.y = -120;
	run();
}

Behavior::~Behavior()
{

}

void Behavior::chooseLevel()
{
	ROS_INFO("Level:\t%d\n",level);
	switch(level){
		case Level_1:
			if(holdBall()){
				std::cout << "Aim\n";
				aim(getTargetGoal());
			}else{
				std::cout << "Chase\n";
				chase(getTargetBall());
			}	
			break;
	}
}

int Behavior::setStatus()
{
	if(status == 0){
		return Halt;
	}else if(status == 1){
		return Start;
	}else{
		return Error;
	}
}

void Behavior::start()
{
	chooseLevel();
}

void Behavior::halt()
{

}

void Behavior::chase(const Ball target_ball)
{
	double distance_x = target_ball.pos.x - env->robot.pos.x;
	double distance_y = target_ball.pos.y - env->robot.pos.y;
	double distance = sqrt(pow(distance_x, 2)+pow(distance_y, 2));
	if(distance > 60){
		double middlePoint_x = -145;
		double middlePoint_y = (env->robot.pos.y+target_ball.pos.y)/2;
		gotoPoint(env, middlePoint_x, middlePoint_y);
	}else{
		chaseBall(env);
	}
}

void Behavior::aim(const Goal target_goal)
{
	double distance_x = target_goal.pos.x - env->robot.pos.x;
	double distance_y = target_goal.pos.y - env->robot.pos.y;
	double distance = sqrt(pow(distance_x, 2)+pow(distance_y, 2));
	double target_angle;
	if(distance_y >= 0){
		target_angle = atan2(distance_x, distance_y)*180/M_PI - env->robot.pos.z;
	}else{
		target_angle = atan2(distance_x, distance_y)*180/M_PI - 360 - env->robot.pos.z;
	}
	if(fabs(target_angle)<1){
		shoot();
	}else{
		aimTargetCone(env, target_angle);
	}
}

void Behavior::shoot()
{
	if(holdBall()){
		env->robot.shoot = 50;
	}else{
		nextTarget();
	}
}

void Behavior::run()
{
	ros::Rate loop_rate(50);
	while(ros::ok()){
		switch(setStatus()){
			case Halt:
				ROS_INFO("status:\tHalt");
				halt();
				break;
			case Start:
				ROS_INFO("status:\tStart");
				if(level==0){
					std::cout << "waiting for subsribe game level\n";
				}else{
					start();
				}
				break;
			case Error:
				ROS_INFO("status:\tHalt");
				halt();
				break;
			default:
				ROS_INFO("status:\tHalt");
				halt();
				break;
		}
		std::cout << "Robot position: (" << env->robot.pos.x << ", " << env->robot.pos.y << ", " << env->robot.pos.z << ")\n";
		std::cout << "Motion feedback(yaw): " << env->robot.pos.angle << std::endl;
		std::cout << "Robot speed: (" << env->robot.v_x << ", " << env->robot.v_y << ", " << env->robot.v_yaw << ")\n";
		std::cout << "---------------------------\n";
		std::cout << "Ball distance: " << env->robot.ball.distance;
		std::cout << "\tBall angle: " << env->robot.ball.angle;
		std::cout << "\n============================\n\n";
		ros::spinOnce();
		loop_rate.sleep();
	}	
}

void Behavior::finish()
{
	ros::shutdown();
}

bool Behavior::holdBall()
{
	if((env->ball.pos.distance<35) && (fabs(env->ball.pos.angle)<3)){
		return true;
	}else{
		return false;
	}
}

void Behavior::nextTarget()
{
	target = target+1;
}

Ball Behavior::getTargetBall()
{
	switch(level){
		case Level1:
			switch(target){
				case Target1:
					return ball_2;
					break;
				default:
					finish();
					exit(1);
			}
			break;
		case Level2:
			switch(target){
				case Target1:
					return ball_1;
					break;
				case Target2:
					return ball_4;
					break;
				default:
					finish();
					exit(1);
			}
			break;
		case Level3:
			switch(target){
				case Target1:
					return ball_2;
					break;
				case Target2:
					return ball_1;
					break;
				case Target3:
					return ball_4;
					break;
				default:
					finish();
					exit(1);
			}
			break;
		case Level4:
			switch(target){
				case Target1:
					return ball_1;
					break;
				case Target2:
					return ball_2;
					break;
				case Target3:
					return ball_3;
					break;
				case Target4:
					return ball_4;
					break;
				default:
					finish();
					exit(1);
			}		
			break;
	}
}

Goal Behavior::getTargetGoal()
{
	switch(level){
		case Level1:
			switch(target){
				case Target1:
					return goal_2;
					break;
				default:
					finish();
					exit(1);
			}
			break;
		case Level2:
			switch(target){
				case Target1:
					return goal_1;
					break;
				case Target2:
					return goal_4;
					break;
				default:
					finish();
					exit(1);
			}
			break;
		case Level3:
			switch(target){
				case Target1:
					return goal_2;
					break;
				case Target2:
					return goal_4;
					break;
				case Target3:
					return goal_1;
					break;
				default:
					finish();
					exit(1);
			}
			break;
		case Level4:
			switch(target){
				case Target1:
					return goal_4;
					break;
				case Target2:
					return goal_3;
					break;
				case Target3:
					return goal_2;
					break;
				case Target4:
					return goal_1;
					break;
				default:
					finish();
					exit(1);
			}		
			break;
	}
}
