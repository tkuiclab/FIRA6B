#include "base_control.h"

//#include "../common/cssl/cssl.h"
#include "../common/cssl/cssl.c"
#include "../common/cssl/port.h"


Base_Control::Base_Control()
{
	baseCMD = new command;
	baseFB = new motor_feedback;
	serial = NULL;
	w1_dir=0,w2_dir=0,w3_dir=0;
	w1_byte=0,w2_byte=0,w3_byte=0;
	en1=0,en2=0,en3=0,stop1=0,stop2=0,stop3=0;
	shoot_byte=0;
	en_byte=0,checksum_byte=0;

	w1_speed=0, w1_speed_percent=0;
	w2_speed=0, w2_speed_percent=0;
	w3_speed=0, w3_speed_percent=0;
	shoot_power=0, shoot_power_percent=0;

	mcssl_init();
}

Base_Control::~Base_Control()
{
	w1_dir=0,w2_dir=0,w3_dir=0;
	w1_byte=0,w2_byte=0,w3_byte=0;
	en1=0,en2=0,en3=0,stop1=0,stop2=0,stop3=0;
	shoot_byte=0;
	en_byte=0,checksum_byte=0;

	w1_speed=0, w1_speed_percent=0;
	w2_speed=0, w2_speed_percent=0;
	w3_speed=0, w3_speed_percent=0;
	shoot_power=0, shoot_power_percent=0;
	
	mcssl_send2motor();
	mcssl_send2motor();
	mcssl_send2motor();
	mcssl_send2motor();
	mcssl_send2motor();
	mcssl_send2motor();
	mcssl_send2motor();
#ifdef DEBUG
	std::cout << "~Base_Control(DEBUG)\n";
#endif
	//delete baseCMD;
	//delete baseFB;
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
	en_byte = 128*en1+64*en2+32*en3+16*stop1+8*stop2+4*stop3;
	checksum_byte = w1_byte+w2_byte+w3_byte+en_byte+shoot_byte;
#ifdef DEBUG_CSSL
	std::cout << "mcssl_send2motor(DEBUG_CSSL)\n";
	std::cout << std::hex;
	std::cout << "w1_byte: " << (int)w1_byte << std::endl;
	std::cout << "w2_byte: " << (int)w2_byte << std::endl;
	std::cout << "w3_byte: " << (int)w3_byte << std::endl;
	std::cout << "en1 en2 en3 stop1 stop2 stop3: ";	
	std::cout << (int)en1 << (int)en2 << (int)en3 << (int)stop1 << " " 
			  << (int)stop2 << (int)stop3 << "00 (" << (int)en_byte << ")"<<  std::endl;
	std::cout << "shoot: " << (int)shoot_byte << std::endl;
	std::cout << "checksum: " << (int)checksum_byte << std::endl;
	std::cout << std::endl;
#else
#ifdef DEBUG
	std::cout << "mcssl_send2motor(DEBUG)\n";
	std::cout << std::hex;
	std::cout << "w1_byte: " << (int)w1_byte << std::endl;
	std::cout << "w2_byte: " << (int)w2_byte << std::endl;
	std::cout << "w3_byte: " << (int)w3_byte << std::endl;
	std::cout << "en1 en2 en3 stop1 stop2 stop3: ";	
	std::cout << (int)en1 << (int)en2 << (int)en3 << (int)stop1 << " " 
			  << (int)stop2 << (int)stop3 << "00 (" << (int)en_byte << ")" << std::endl;
	std::cout << "shoot: " << (int)shoot_byte << std::endl;
	std::cout << "checksum: " << (int)checksum_byte << std::endl;
	std::cout << std::endl;
#endif
	cssl_putchar(serial, 0xff);
	cssl_putchar(serial, 0xfa);
	cssl_putchar(serial, w1_byte);
	cssl_putchar(serial, w2_byte);
	cssl_putchar(serial, w3_byte);
	cssl_putchar(serial, en_byte);
	cssl_putchar(serial, shoot_byte);
	cssl_putchar(serial, checksum_byte);
#endif
}

void Base_Control::shoot_regularization()
{
	if(shoot_power_percent>100)shoot_power_percent=100;
	shoot_power = 255*shoot_power_percent/100;
	shoot_byte = (unsigned char)shoot_power;
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
	w1_byte = (w1_speed_percent>0)? (unsigned char)(127*w1_speed_percent/100) + 12.7 + w1_dir : 0;
	w2_byte = (w2_speed_percent>0)? (unsigned char)(127*w2_speed_percent/100) + 12.7 + w2_dir : 0;
	w3_byte = (w3_speed_percent>0)? (unsigned char)(127*w3_speed_percent/100) + 12.7 + w3_dir : 0;
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
	w1_speed = baseCMD->x_speed*cos(m1_Angle)+baseCMD->y_speed*sin(m1_Angle)+baseCMD->yaw_speed*robot_radius*(-1);
	w2_speed = baseCMD->x_speed*cos(m2_Angle)+baseCMD->y_speed*sin(m2_Angle)+baseCMD->yaw_speed*robot_radius*(-1);
	w3_speed = baseCMD->x_speed*cos(m3_Angle)+baseCMD->y_speed*sin(m3_Angle)+baseCMD->yaw_speed*robot_radius*(-1);
	shoot_power_percent = baseCMD->shoot_power;

	for(int i=0;i<10;i++){
		if((w1_speed>100)||(w2_speed>100)||(w3_speed>100)){
			w1_speed = w1_speed*0.9;
			w2_speed = w2_speed*0.9;
			w3_speed = w3_speed*0.9;
		}else{
			w1_speed = w1_speed;
			w2_speed = w2_speed;
			w3_speed = w3_speed;
			break;
		}
	}
#ifdef DEBUG
	std::cout << "Inverse kinematics(DEBUG)\n";
	std::cout << "x_speed: " << baseCMD->x_speed << std::endl;
	std::cout << "y_speed: " << baseCMD->y_speed << std::endl;
	std::cout << "yaw_speed: " << baseCMD->yaw_speed << std::endl;
	std::cout << "w1_speed(%): " << w1_speed << std::endl;
	std::cout << "w2_speed(%): " << w2_speed << std::endl;
	std::cout << "w3_speed(%): " << w3_speed << std::endl;
	std::cout << std::endl;
#endif
}
void Base_Control::send(command* CMD)
{
	baseCMD = CMD;
	inverseKinematics();
	shoot_regularization();
	speed_regularization();
	mcssl_send2motor();
}

