#ifndef MOTOR_DATA_H
#define MOTOR_DATA_H

typedef struct{
	void* x_speed;
	void* y_speed;
	void* yaw_speed;
	void* shoot_power;
}robot_command;

typedef struct{
	void* w1_speed;
	void* w2_speed;
	void* w3_speed;
	void* enable_stop;
}motor_command;

typedef struct{
	void* motor1_feedback;
	void* motor2_feedback;
	void* motor3_feedback;
}motor_feedback;

//typedef struct MOTOR_FEEDBACK motor_feedback;
//typedef struct MOTOR_COMMAND motor_command;
#endif 

