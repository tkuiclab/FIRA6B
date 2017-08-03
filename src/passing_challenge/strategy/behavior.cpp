#include "behavior.h"

Behavior::Behavior(int argc, char** argv)
{
	//nn = new Strategy_nodeHandle();
	env = getEnv();
	init(argc, argv);
	target = 1;
	ball_1.pos.x = -1.65;
	ball_1.pos.y = 1.20;
	ball_2.pos.x = -1.65;
	ball_2.pos.y = 0.40;
	ball_3.pos.x = -1.65;
	ball_3.pos.y = -0.40;
	ball_4.pos.x = -1.65;
	ball_4.pos.y = -1.20;
	
	goal_1.pos.x = 1.65;
	goal_1.pos.y = 1.20;
	goal_2.pos.x = 1.65;
	goal_2.pos.y = 0.40;
	goal_3.pos.x = 1.65;
	goal_3.pos.y = -0.40;
	goal_4.pos.x = 1.65;
	goal_4.pos.y = -1.20;
	run();
}

Behavior::~Behavior()
{

}

void Behavior::chooseLevel()
{
    printf("Level: %d, Target: %d\n",level,target);
	if(holdBall() == true){
		std::cout << "Aim\n";
		aim(getTargetGoal());
	}else{
		std::cout << "Chase\n";
		chase(getTargetBall());
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
	double target_angle = (atan2(distance_y, distance_x)*180/M_PI)-(env->robot.pos.z);
	angleRegular(target_angle);
    if(distance > 0.6){
		double middlePoint_x = (target_ball.pos.x+env->robot.pos.x)/2;
        double middlePoint_y = (target_ball.pos.y+env->robot.pos.y)/2;
//		double middlePoint_y = (env->robot.pos.y+target_ball.pos.y)/2;
		std::cout << "Target Ball Position: (" << target_ball.pos.x << ", " << target_ball.pos.y << ", " << target_angle << ")\n";
		std::cout << "Target Ball Distance: " << distance << std::endl;
		std::cout << "Target Point: (" << middlePoint_x <<  ", " << middlePoint_y << ")\n";
		gotoPoint(env, middlePoint_x, middlePoint_y, target_angle);
	}else{
		std::cout << "Got Ball\nTarget Ball Distance: " << env->robot.ball.distance << "\tTarget Ball Angle: " << env->robot.ball.angle << "\n";
		chaseBall(env);
	}
	pub();
}

void Behavior::aim(const Goal target_goal)
{
	double distance_x = target_goal.pos.x - env->robot.pos.x;
	double distance_y = target_goal.pos.y - env->robot.pos.y;
	double distance = sqrt(pow(distance_x, 2)+pow(distance_y, 2));
	double target_angle = (atan2(distance_y, distance_x)*180/M_PI)-(env->robot.pos.z);
	angleRegular(target_angle);
	std::cout << "Target Goal: (" << target_goal.pos.x << ", " << target_goal.pos.y << ", " << target_angle << " )\n";
	std::cout << "Target Goal Distace: " << distance << std::endl;	
    if(distance > 2.0){
        double middlePoint_x = 0.0;
        double middlePoint_y = target_goal.pos.y;
        gotoPoint(env, middlePoint_x, middlePoint_y, target_angle);
    }else if(fabs(target_angle)<1){
		pub(0);
		sleep(1);
		ros::spinOnce();
		distance_x = target_goal.pos.x - env->robot.pos.x;
		distance_y = target_goal.pos.y - env->robot.pos.y;
		target_angle = (atan2(distance_y, distance_x)*180/M_PI)-(env->robot.pos.z);
		angleRegular(target_angle);
		if(fabs(target_angle)<1)	shoot();
		else	aimTargetCone(env, target_angle);

	}else{
		aimTargetCone(env, target_angle);
	}
	pub();
}

void Behavior::shoot()
{
	std::cout << "------->Shoot\n";
	env->robot.v_x = 0;
	env->robot.v_y = 0;
	env->robot.v_yaw = 0;
	env->robot.shoot = 50;
	pub();	
	sleep(1);
	nextTarget();
}

void Behavior::run()
{
	geometry_msgs::Twist motion;
	std_msgs::Int32 shoot;
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
					if(env->robot.shoot>0)env->robot.shoot = 0;
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
    if((env->robot.ball.distance<=0.36) && (fabs(env->robot.ball.angle)<=3)){
		std::cout << "got ball\n";
		return true;
	}else{
		std::cout << "lose ball\n";
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

void Behavior::angleRegular(double &angle)
{
	if(angle>180)angle = angle-360;
	else if(angle<=(-180))angle = angle+360;
	else angle = angle;
}
