#ifndef MOTOR_DATA_H
#define MOTOR_DATA_H

typedef struct{
	double x_speed;
	double y_speed;
	double yaw_speed;
	int shoot_power;
}command;

typedef struct{
	void *w1_speed;
	void *w2_speed;
	void *w3_speed;
}motor_speed;

typedef struct{
	void* motor1_feedback;
	void* motor2_feedback;
	void* motor3_feedback;
}motor_feedback;

//typedef struct MOTOR_FEEDBACK motor_feedback;
//typedef struct MOTOR_COMMAND motor_command;
#endif 

