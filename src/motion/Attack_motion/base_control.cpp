#include "base_control.h"

//#include "../common/cssl/cssl.h"
#include "../common/cssl/cssl.c"
#include "../common/cssl/port.h"


Base_Control::Base_Control()
{
#ifdef DEBUG
	std::cout << "Base_Control(DEBUG)\n";
#endif
	this->base_robotCMD = new robot_command;
	this->base_motorCMD = new motor_command;
	//this->base_motorCMD_percent = new motor_command;
	//this->base_motorCMD_byte = new motor_command;
	this->baseFB = new motor_feedback;
	
	this->base_robotCMD->x_speed = new double;
	this->base_robotCMD->y_speed = new double;
	this->base_robotCMD->yaw_speed = new double;
	this->base_robotCMD->shoot_power = new int;

	this->base_motorCMD->w1_speed = new unsigned char;
	this->base_motorCMD->w2_speed = new unsigned char;
	this->base_motorCMD->w3_speed = new unsigned char;
	this->base_motorCMD->enable_stop = new unsigned char;
	
	
	this->w1_speed=0;this->w1_speed_percent=0;
	this->w2_speed=0;this->w2_speed_percent=0;
	this->w3_speed=0;this->w3_speed_percent=0;
	this->shoot_power=0;this->shoot_power_percent=0;
	this->shoot_byte=0;
	this->checksum_byte=0;
	this->serial = NULL;
	//w1_dir=0,w2_dir=0,w3_dir=0;
	//w1_byte=0,w2_byte=0,w3_byte=0;
	//en1=0,en2=0,en3=0,stop1=0,stop2=0,stop3=0;

	//w1_speed=0, w1_speed_percent=0;
	//w2_speed=0, w2_speed_percent=0;
	//w3_speed=0, w3_speed_percent=0;
	//shoot_power=0, shoot_power_percent=0;
	std::cout << "Init base control\n";
	mcssl_init();
}

Base_Control::~Base_Control()
{
	//w1_dir=0,w2_dir=0,w3_dir=0;
	//w1_byte=0,w2_byte=0,w3_byte=0;
	//en1=0,en2=0,en3=0,stop1=0,stop2=0,stop3=0;
	//shoot_byte=0;
	//en_byte=0,checksum_byte=0;

	//w1_speed=0, w1_speed_percent=0;
	//w2_speed=0, w2_speed_percent=0;
	//w3_speed=0, w3_speed_percent=0;
	//shoot_power=0, shoot_power_percent=0;
	
	//mcssl_send2motor();
#ifdef DEBUG
	std::cout << "~Base_Control(DEBUG)\n";
#endif
	mcssl_finish();
}

int Base_Control::mcssl_init()
{
	char *devs;
#ifdef DEBUG_CSSL
	std::cout << "mcssl_init(DEBUG_CSSL)\n";
#else
	cssl_start();
	if(!serial){
		devs = "/dev/ttyUSB0";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB1";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB2";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB3";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB4";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB5";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB6";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB7";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB8";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB9";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB10";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		std::cout << cssl_geterrormsg() << std::endl;
		std::cout << "----> ATTACK MOTION RS232 OPEN FAILED <----\n";
		fflush(stdout);
		//return 0;
		std::cout << devs << std::endl;
		exit(EXIT_FAILURE);
	}else{
		std::cout << "----> ATTACK MOTION RS232 OPEN SUCCESSFUL <----\n";
		std::cout << "Initialize attack motion with port = "<< devs << "...\n";
		cssl_setflowcontrol(serial, 0, 0);
	}
#endif
	return 1;
}

void Base_Control::mcssl_finish()
{
#ifdef DEBUG_CSSL
	std::cout << "mcssl_finish(DEBUG_CSSL)\n";
#else
	cssl_close(serial);
	cssl_stop();
#endif
}

void Base_Control::mcssl_Callback(int id, uint8_t *buf, int length)
{
#ifdef DEBUG_CSSLCALLBACK
	std::cout << "mcssl_Callback(DEBUG_CSSLCALLBACK)\n";
#else
#endif
}

void Base_Control::mcssl_send2motor()
{	
	checksum_byte = *(unsigned char*)this->base_motorCMD->w1_speed+*(unsigned char*)this->base_motorCMD->w2_speed+*(unsigned char*)this->base_motorCMD->w3_speed+*(unsigned char*)this->base_motorCMD->enable_stop+this->shoot_byte;
#ifdef DEBUG_CSSL
	std::cout << "mcssl_send2motor(DEBUG_CSSL)\n";
	std::cout << std::hex;
	std::cout << "w1_byte: " << (int)*(unsigned char*)this->base_motorCMD->w1_speed << std::endl;
	std::cout << "w2_byte: " << (int)*(unsigned char*)this->base_motorCMD->w2_speed << std::endl;
	std::cout << "w3_byte: " << (int)*(unsigned char*)this->base_motorCMD->w3_speed << std::endl;
	std::cout << "en1 en2 en3 stop1 stop2 stop3: ";	
	std::cout << (int)en1 << (int)en2 << (int)en3 << (int)stop1;
	std::cout << " ";
	std::cout << (int)stop2 << (int)stop3 << "00 (" << (int)*(unsigned char*)this->base_motorCMD->enable_stop << ")"<<  std::endl;
	std::cout << "shoot: " << (int)shoot_byte << std::endl;
	std::cout << "checksum: " << (int)checksum_byte << std::endl;
	std::cout << std::endl;
#else
#ifdef DEBUG
	std::cout << "mcssl_send2motor(DEBUG)\n";
	std::cout << std::hex;
	std::cout << "w1_byte: " << (int)*(unsigned char*)this->base_motorCMD->w1_speed << std::endl;
	std::cout << "w2_byte: " << (int)*(unsigned char*)this->base_motorCMD->w2_speed << std::endl;
	std::cout << "w3_byte: " << (int)*(unsigned char*)this->base_motorCMD->w3_speed << std::endl;
	std::cout << "en1 en2 en3 stop1 stop2 stop3: ";	
	std::cout << (int)en1 << (int)en2 << (int)en3 << (int)stop1 << " " 
			  << (int)stop2 << (int)stop3 << "00 (" << (int)*(unsigned char*)this->base_motorCMD->enable_stop << ")"<<  std::endl;
	std::cout << "shoot: " << (int)shoot_byte << std::endl;
	std::cout << "checksum: " << (int)checksum_byte << std::endl;
	std::cout << std::endl;
#endif
	cssl_putchar(serial, 0xff);
	cssl_putchar(serial, 0xfa);
	cssl_putchar(serial, *(unsigned char*)this->base_motorCMD->w1_speed);
	cssl_putchar(serial, *(unsigned char*)this->base_motorCMD->w2_speed);
	cssl_putchar(serial, *(unsigned char*)this->base_motorCMD->w3_speed);
	cssl_putchar(serial, *(unsigned char*)this->base_motorCMD->enable_stop);
	cssl_putchar(serial, shoot_byte);
	cssl_putchar(serial, checksum_byte);
#endif
}

void Base_Control::shoot_regularization()
{
	if(*(int*)this->base_robotCMD->shoot_power>=100)shoot_byte = 255;
	else{
		shoot_byte = (unsigned char)(255*(*(int*)this->base_robotCMD->shoot_power)/100);
	}
#ifdef DEBUG
	std::cout << "shoot_regularization(DEBUG)\n";
	std::cout << std::hex;
	std::cout << "shoot byte(hex): " << (int)this->shoot_byte << std::endl;
	std::cout << std::endl;
#endif
}

void Base_Control::speed_regularization()
{
	w1_dir = (w1_speed>0)? 0x80 : 0;
	w2_dir = (w2_speed>0)? 0x80 : 0;
	w3_dir = (w3_speed>0)? 0x80 : 0;

	w1_speed_percent = (fabs(w1_speed)<1)? 0 : fabs(w1_speed);
	w2_speed_percent = (fabs(w2_speed)<1)? 0 : fabs(w2_speed);
	w3_speed_percent = (fabs(w3_speed)<1)? 0 : fabs(w3_speed);
//	enable
	en1 = (w1_speed_percent > 0)? 1 : 0;
	en2 = (w2_speed_percent > 0)? 1 : 0;
	en3 = (w3_speed_percent > 0)? 1 : 0;
//	stop
/*
	if((w1_speed_percent != 999)&&(stop1 == 1)){
		en1 = 0;
	}else{
		en1 = ((stop1 == 1) || (w1_byte > 0))? 1 : 0;
	}
	if((w2_speed_percent != 999)&&(stop2 == 1)){
		en2 = 0;
	}else{
		en2 = ((stop2 == 1) || (w2_byte > 0))? 1 : 0;
	}
	if((w3_speed_percent != 999)&&(stop3 == 1)){
		en3 = 0;
	}else{
		en3 = ((stop3 == 1) || (w3_byte > 0))? 1 : 0;
	}
*/
//	speed -> speed_byte
	*(unsigned char*)this->base_motorCMD->w1_speed = (w1_speed_percent>0)? (unsigned char)((127*0.8*w1_speed_percent/100) + 12.7 + w1_dir) : 0;
	*(unsigned char*)this->base_motorCMD->w2_speed = (w2_speed_percent>0)? (unsigned char)((127*0.8*w2_speed_percent/100) + 12.7 + w2_dir) : 0;
	*(unsigned char*)this->base_motorCMD->w3_speed = (w3_speed_percent>0)? (unsigned char)((127*0.8*w3_speed_percent/100) + 12.7 + w3_dir) : 0;
	*(unsigned char*)this->base_motorCMD->enable_stop = (this->en1<<7)+(this->en2<<6)+(this->en3<<5)+(this->stop1<<4)+(this->stop2<<3)+(this->stop3<<2);
#ifdef DEBUG
	std::cout << "speed_regularization(DEBUG)\n";
	std::cout << std::hex;
	std::cout << "motor1 speed(hex): " << (int)*(unsigned char*)this->base_motorCMD->w1_speed << std::endl;
	std::cout << "motor2 speed(hex): " << (int)*(unsigned char*)this->base_motorCMD->w2_speed << std::endl;
	std::cout << "motor3 speed(hex): " << (int)*(unsigned char*)this->base_motorCMD->w3_speed << std::endl;
	std::cout << std::hex;
	std::cout << "enable & stop(hex): " << (int)*(unsigned char*)this->base_motorCMD->enable_stop << std::endl;
	std::cout << std::endl;
#endif
/*
	if(w1_speed_percent == 999){
		w1_speed = 0;
		stop1 = 1;
	}else{
		w1_speed = 127*(0.8*fabs(w1_speed_percent));
		w1_speed = (w1_speed>0)? w1_speed+12.7 : 0;
		w1_byte = (unsigned char)w1_speed+w1_dir;
		stop1 = 0;
	}
	if(w2_speed_percent == 999){
		w2_speed = 0;
		stop2 = 1;
	}else{
		w2_speed = 127*(0.8*fabs(w2_speed_percent));
		w2_speed = (w2_speed>0)? w2_speed+12.7 : 0;
		w2_byte = (unsigned char)w2_speed+w2_dir;
		stop2 = 0;
	}
	if(w3_speed_percent == 999){
		w3_speed = 0;
		stop3 = 1;
	}else{
		w3_speed = 127*(0.8*fabs(w3_speed_percent));
		w3_speed = (w3_speed>0)? w3_speed+12.7 : 0;
		w3_byte = (unsigned char)w3_speed+w3_dir;
		stop3 = 0;
	}
*/
}

void Base_Control::inverseKinematics()
{
	this->w1_speed = *(double*)this->base_robotCMD->x_speed*cos(m1_Angle)+*(double*)this->base_robotCMD->y_speed*sin(m1_Angle)+*(double*)this->base_robotCMD->yaw_speed*robot_radius*(-1);
	this->w2_speed = *(double*)this->base_robotCMD->x_speed*cos(m2_Angle)+*(double*)this->base_robotCMD->y_speed*sin(m2_Angle)+*(double*)this->base_robotCMD->yaw_speed*robot_radius*(-1);
	this->w3_speed = *(double*)this->base_robotCMD->x_speed*cos(m3_Angle)+*(double*)this->base_robotCMD->y_speed*sin(m3_Angle)+*(double*)this->base_robotCMD->yaw_speed*robot_radius*(-1);

	for(int i=0;i<10;i++){
		if(fabs(this->w1_speed)>100||fabs(this->w2_speed)>100||fabs(this->w3_speed>100)){
			this->w1_speed = this->w1_speed*0.9;
			this->w2_speed = this->w2_speed*0.9;
			this->w3_speed = this->w3_speed*0.9;
		}else{
			this->w1_speed = this->w1_speed;
			this->w2_speed = this->w2_speed;
			this->w3_speed = this->w3_speed;
			break;
		}
	}
#ifdef DEBUG
	std::cout << "Inverse kinematics(DEBUG)\n";
	std::cout << std::dec;
	std::cout << "x_speed: " << *(double*)base_robotCMD->x_speed << std::endl;
	std::cout << "y_speed: " << *(double*)base_robotCMD->y_speed << std::endl;
	std::cout << "yaw_speed: " << *(double*)base_robotCMD->yaw_speed << std::endl;
	std::cout << "w1_speed(%): " << this->w1_speed << std::endl;
	std::cout << "w2_speed(%): " << this->w2_speed << std::endl;
	std::cout << "w3_speed(%): " << this->w3_speed << std::endl;
	std::cout << std::endl;
#endif
}
void Base_Control::send(robot_command* CMD)
{
	this->base_robotCMD = CMD;
	inverseKinematics();
	shoot_regularization();
	speed_regularization();
	mcssl_send2motor();
}

