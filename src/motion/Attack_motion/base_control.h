#ifndef Base_Control_H
#define Base_Control_H
/*******************************
  * Include system
  ******************************/
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
/*******************************
  * Include library
  ******************************/
#include "../common/motor_data.h"
#include "../common/cssl/cssl.h"
//#include "../common/cssl/cssl.c"
//#include "../common/cssl/port.h"

/*******************************
  * Define 
  ******************************/
//#define DEBUG
//#define DEBUG_CSSL

class Base_Control{
public:
	Base_Control();
	~Base_Control();

private:
	void	mcssl_Callback(int id, uint8_t *buf, int length);
	void 	mcssl_finish();
	void 	mcssl_send2motor();
	int 	mcssl_init();
	void	shoot_regularization();
	void	speed_regularization();
	void	inverseKinematics();
private:
	const double m1_Angle = -M_PI/3;
	const double m2_Angle =  M_PI/3;
	const double m3_Angle = -M_PI;
	const double robot_radius = 0.15;
	const double wheel_radius = 0.00508;


	cssl_t *serial;
	unsigned char w1_dir,w2_dir,w3_dir;
	unsigned char w1_byte,w2_byte,w3_byte;
	unsigned char en1,en2,en3,stop1,stop2,stop3;
	unsigned char shoot_byte;
	unsigned char en_byte,checksum_byte;

	command	*baseCMD;
	motor_feedback 	*baseFB;
	double w1_speed, w1_speed_percent;
	double w2_speed, w2_speed_percent;
	double w3_speed, w3_speed_percent;
	int shoot_power, shoot_power_percent;
	//void send();
	//void get();
public:
	void send(command*);
//	int 	mcssl_init();
};
#endif
