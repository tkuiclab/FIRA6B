#ifndef MOTOR_DATA_H
#define MOTOR_DATA_H

typedef struct{
	double x_speed;
	double y_speed;
	double yaw_speed;
	int shoot_power;
}motor_command;
typedef struct{
	double* motor1_feedback;
	double* motor2_feedback;
	double* motor3_feedback;
}motor_feedback;

//typedef struct MOTOR_FEEDBACK motor_feedback;
//typedef struct MOTOR_COMMAND motor_command;
#endif 

