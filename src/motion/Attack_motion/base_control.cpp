#include "base_control.h"

//#include "../common/cssl/cssl.h"
#include "cssl.c"
#include "port.h"
serial_rx* Base_Control::base_RX = NULL;
Base_Control::Base_Control()
{
	this->base_robotFB = new robot_command;
	this->base_robotFB->x_speed = new double;
	this->base_robotFB->y_speed = new double;
	this->base_robotFB->yaw_speed = new double;
	this->base_robotFB->shoot_power = new int;
	std::memset(this->base_robotFB->x_speed, 0, sizeof(double));
	std::memset(this->base_robotFB->y_speed, 0, sizeof(double));
	std::memset(this->base_robotFB->yaw_speed, 0, sizeof(double));
	std::memset(this->base_robotFB->shoot_power, 0, sizeof(int));

	Base_Control::base_RX = new serial_rx;

	Base_Control::base_RX->head1 = new unsigned char;
	Base_Control::base_RX->head2 = new unsigned char;
	Base_Control::base_RX->w1 = new int;
	Base_Control::base_RX->w2 = new int;
	Base_Control::base_RX->w3 = new int;
	Base_Control::base_RX->shoot = new unsigned char;
	Base_Control::base_RX->batery = new unsigned char;
	memset(Base_Control::base_RX->head1, 0, sizeof(unsigned char));
	memset(Base_Control::base_RX->head2, 0, sizeof(unsigned char));
	memset(Base_Control::base_RX->w1, 0, sizeof(int));
	memset(Base_Control::base_RX->w2, 0, sizeof(int));
	memset(Base_Control::base_RX->w3, 0, sizeof(int));
	memset(Base_Control::base_RX->shoot, 0, sizeof(unsigned char));
	memset(Base_Control::base_RX->batery, 0, sizeof(unsigned char));

	this->base_TX = new serial_tx;

	this->base_TX->head1 = new unsigned char;
	this->base_TX->head2 = new unsigned char;
	this->base_TX->w1 = new unsigned char;
	this->base_TX->w2 = new unsigned char;
	this->base_TX->w3 = new unsigned char;
	this->base_TX->w4 = new unsigned char;
	this->base_TX->enable_and_stop = new unsigned char;
	this->base_TX->shoot = new unsigned char;
	this->base_TX->checksum = new unsigned char;

	memset(this->base_TX->head1, 0xff, sizeof(unsigned char));
	memset(this->base_TX->head2, 0xfa, sizeof(unsigned char));
	memset(this->base_TX->w1, 0, sizeof(unsigned char));
	memset(this->base_TX->w2, 0, sizeof(unsigned char));
	memset(this->base_TX->w3, 0, sizeof(unsigned char));
	memset(this->base_TX->w4, 0, sizeof(unsigned char));
	memset(this->base_TX->enable_and_stop, 0, sizeof(unsigned char));
	memset(this->base_TX->shoot, 0, sizeof(unsigned char));
	memset(this->base_TX->checksum, 0, sizeof(unsigned char));

	this->serial = NULL;
	//Base_Control::count_buffer = 0;
	//memset(Base_Control::cssl_buffer, 0, sizeof(unsigned char));
#ifdef DEBUG
	std::cout << "Base_Control(DEBUG)\n";
	std::cout << "Init base control\n";
	/*std::cout << (int)*this->base_TX->head1 << std::endl;
	std::cout << (int)*this->base_TX->head2 << std::endl;
	std::cout << (int)*this->base_TX->w1 << std::endl;
	std::cout << (int)*this->base_TX->w2 << std::endl;
	std::cout << (int)*this->base_TX->w3 << std::endl;
	std::cout << (int)*this->base_TX->enable_and_stop << std::endl;
	std::cout << (int)*this->base_TX->shoot << std::endl;
	std::cout << (int)*this->base_TX->checksum << std::endl;*/
#endif
	mcssl_init();
}

Base_Control::~Base_Control()
{
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
	std::cout << "test\n";
	if(!serial){
		devs = "/dev/ttyUSB0";
		serial = cssl_open(devs, mcssl_Callback/*NULL*/, 0, 115200, 8, 0, 1);
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
	//serial->callback = &Base_Control::mcssl_Callback;
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

void Base_Control::mcssl_Callback(int id, uint8_t* buf, int length)
{
	static unsigned char cssl_buffer[50]={0};
	static int count_buffer=0;
	cssl_buffer[count_buffer++] = *buf;
	count_buffer = (count_buffer)%50;
	unsigned char checksum;
	int i;
	for(i=0; i<30; i++){
		if((cssl_buffer[i]==0xff)&&(cssl_buffer[i+1]==0xfa)){
			checksum = cssl_buffer[i+2]+cssl_buffer[i+3]+cssl_buffer[i+4]+cssl_buffer[i+5]+cssl_buffer[i+6]+cssl_buffer[i+7]+cssl_buffer[i+8]+cssl_buffer[i+9]+cssl_buffer[i+10]+cssl_buffer[i+11]+cssl_buffer[i+12]+cssl_buffer[i+13];
			if(cssl_buffer[i+14]==checksum){
				*(base_RX->head1) = cssl_buffer[i];
				*(base_RX->head2) = cssl_buffer[i+1];
				*(base_RX->w1) = (cssl_buffer[i+2]<<24)+(cssl_buffer[i+3]<<16)+(cssl_buffer[i+4]<<8)+(cssl_buffer[i+5]);
				*(base_RX->w2) = (cssl_buffer[i+6]<<24)+(cssl_buffer[i+7]<<16)+(cssl_buffer[i+8]<<8)+(cssl_buffer[i+9]);
				*(base_RX->w3) = (cssl_buffer[i+10]<<24)+(cssl_buffer[i+11]<<16)+(cssl_buffer[i+12]<<8)+(cssl_buffer[i+13]);
				*(base_RX->shoot) = 0;
				*(base_RX->batery) = 0;
				break;
			}else{
				continue;
			}
		}else{
			continue;
		}
	}
#ifdef DEBUG_CSSLCALLBACK
	std::cout << "mcssl_Callback(DEBUG_CSSLCALLBACK)\n";
	std::cout << std::hex;
//	std::cout << "buf: " << (int)*(buf) << "\n";
	std::cout << "RX: ";
	for(int j=i; j<i+15; j++){
		std::cout << (int)cssl_buffer[j] << " ";
	}
	std::cout << std::endl;
	std::cout << "head1: " << (int)*(base_RX->head1) << "\n";
	std::cout << "head2: " << (int)*(base_RX->head2) << "\n";
	std::cout << "w1: " << (int)*(base_RX->w1) << "\n";
	std::cout << "w2: " << (int)*(base_RX->w2) << "\n";
	std::cout << "w3: " << (int)*(base_RX->w3) << "\n";
	std::cout << "shoot: " << (int)*(base_RX->shoot) << "\n";
	std::cout << "batery: " << (int)*(base_RX->batery) << "\n";
	std::cout << std::endl;
#else
#endif

}

void Base_Control::mcssl_send2motor()
{	
	*(this->base_TX->checksum) = *(this->base_TX->w1)+*(this->base_TX->w2)+*(this->base_TX->w3)+*(this->base_TX->enable_and_stop)+*(this->base_TX->shoot);
#ifdef DEBUG_CSSL
	std::cout << "mcssl_send2motor(DEBUG_CSSL)\n";
	std::cout << std::hex;
	std::cout << "w1: " << (int)*(this->base_TX->w1) << std::endl;
	std::cout << "w2: " << (int)*(this->base_TX->w2) << std::endl;
	std::cout << "w3: " << (int)*(this->base_TX->w3) << std::endl;
	std::cout << "en1 en2 en3 stop1 stop2 stop3: ";	
	std::cout << (int)en1 << (int)en2 << (int)en3 << (int)stop1;
	std::cout << " ";
	std::cout << (int)stop2 << (int)stop3 << "00 (" << (int)*(this->base_TX->enable_and_stop) << ")"<<  std::endl;
	std::cout << "shoot: " << (int)*(this->base_TX->shoot) << std::endl;
	std::cout << "checksum: " << (int)*(this->base_TX->checksum) << std::endl;
	std::cout << std::endl;
#else
#ifdef DEBUG
	std::cout << "mcssl_send2motor(DEBUG)\n";
	std::cout << std::hex;
	std::cout << "w1: " << (int)*(this->base_TX->w1) << std::endl;
	std::cout << "w2: " << (int)*(this->base_TX->w2) << std::endl;
	std::cout << "w3: " << (int)*(this->base_TX->w3) << std::endl;
	std::cout << "en1 en2 en3 stop1 stop2 stop3: ";	
	std::cout << (int)en1 << (int)en2 << (int)en3 << (int)stop1;
	std::cout << " ";
	std::cout << (int)stop2 << (int)stop3 << "00 (" << (int)*(this->base_TX->enable_and_stop) << ")"<<  std::endl;
	std::cout << "shoot: " << (int)*(this->base_TX->shoot) << std::endl;
	std::cout << "checksum: " << (int)*(this->base_TX->checksum) << std::endl;
	std::cout << std::endl;
#endif
	cssl_putchar(serial, *(this->base_TX->head1));
	cssl_putchar(serial, *(this->base_TX->head2));
	cssl_putchar(serial, *(this->base_TX->w1));
	cssl_putchar(serial, *(this->base_TX->w2));
	cssl_putchar(serial, *(this->base_TX->w3));
	cssl_putchar(serial, *(this->base_TX->enable_and_stop));
	cssl_putchar(serial, *(this->base_TX->shoot));
	cssl_putchar(serial, *(this->base_TX->checksum));
	/*cssl_putchar(serial, 0xff);
	cssl_putchar(serial, 0xfa);
	cssl_putchar(serial, 0);
	cssl_putchar(serial, 0);
	cssl_putchar(serial, 0);
	cssl_putchar(serial, 0x80);
	cssl_putchar(serial, 0x0);*/
#endif
}

void Base_Control::shoot_regularization()
{
	if(*(this->base_robotCMD->shoot_power)>=100){
		*(this->base_TX->shoot) = 255;
	}
	else{
		*(this->base_TX->shoot) = (255**(this->base_robotCMD->shoot_power)/100);
	}
#ifdef DEBUG
	std::cout << "shoot_regularization(DEBUG)\n";
	std::cout << std::hex;
	std::cout << "shoot byte(hex): " << (int)*(this->base_TX->shoot) << std::endl;
	std::cout << std::endl;
#endif
}

void Base_Control::speed_regularization(double w1, double w2, double w3)
{
	unsigned char w1_dir = (w1>0)? 0x80 : 0;
	unsigned char w2_dir = (w2>0)? 0x80 : 0;
	unsigned char w3_dir = (w3>0)? 0x80 : 0;

	double w1_speed_percent = (fabs(w1)<0.5)? 0 : fabs(w1);
	double w2_speed_percent = (fabs(w2)<0.5)? 0 : fabs(w2);
	double w3_speed_percent = (fabs(w3)<0.5)? 0 : fabs(w3);
//	enable
	this->en1 = (w1_speed_percent > 0)? 1 : 0;
	this->en2 = (w2_speed_percent > 0)? 1 : 0;
	this->en3 = (w3_speed_percent > 0)? 1 : 0;
	this->stop1 = 0;
	this->stop2 = 0;
	this->stop3 = 0;
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
	
	*(this->base_TX->w1) = (w1_speed_percent>0)? (unsigned char)((127*0.8*w1_speed_percent/100) + 12.7 + w1_dir) : 0;
	*(this->base_TX->w2) = (w2_speed_percent>0)? (unsigned char)((127*0.8*w2_speed_percent/100) + 12.7 + w2_dir) : 0;
	*(this->base_TX->w3) = (w3_speed_percent>0)? (unsigned char)((127*0.8*w3_speed_percent/100) + 12.7 + w3_dir) : 0;
	*(this->base_TX->enable_and_stop) = (this->en1<<7)+(this->en2<<6)+(this->en3<<5)+(this->stop1<<4)+(this->stop2<<3)+(this->stop3<<2);
#ifdef DEBUG
	std::cout << "speed_regularization(DEBUG)\n";
	std::cout << std::hex;
	std::cout << "motor1 speed(hex): " << (int)*(this->base_TX->w1) << std::endl;
	std::cout << "motor2 speed(hex): " << (int)*(this->base_TX->w2) << std::endl;
	std::cout << "motor3 speed(hex): " << (int)*(this->base_TX->w3) << std::endl;
	std::cout << std::hex;
	std::cout << "enable & stop(hex): " << (int)*(this->base_TX->enable_and_stop) << std::endl;
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

void Base_Control::forwardKinematics()
{
	double yaw=0;
	int round=0;
	*(this->base_robotFB->x_speed) = (*(base_RX->w1)*(-0.3333) + *(base_RX->w2)*(-0.3333) + *(base_RX->w3)*(0.6667))*2*M_PI*wheel_radius/(26)/2000;
    *(this->base_robotFB->y_speed) = (*(base_RX->w1)*(0.5774) + *(base_RX->w2)*(-0.5774) + *(base_RX->w3)*(0))*2*M_PI*wheel_radius/26/2000;
   yaw = (*(base_RX->w1)*(yaw_inv) + *(base_RX->w2)*(yaw_inv) + *(base_RX->w3)*(yaw_inv))*wheel_radius/2000/26;
   round = yaw/360;
   yaw = yaw-(360*round);
   if(yaw>180){
	   *(this->base_robotFB->yaw_speed) = yaw - 360;
   }else if(yaw < -180){
	   *(this->base_robotFB->yaw_speed) = yaw + 360;
   }
}

void Base_Control::inverseKinematics()
{
	double w1_speed, w2_speed, w3_speed;

	w1_speed = *(this->base_robotCMD->x_speed)*cos(m1_Angle)+*(this->base_robotCMD->y_speed)*sin(m1_Angle)+*(this->base_robotCMD->yaw_speed)*robot_radius*(-1);
	w2_speed = *(this->base_robotCMD->x_speed)*cos(m2_Angle)+*(this->base_robotCMD->y_speed)*sin(m2_Angle)+*(this->base_robotCMD->yaw_speed)*robot_radius*(-1);
	w3_speed = *(this->base_robotCMD->x_speed)*cos(m3_Angle)+*(this->base_robotCMD->y_speed)*sin(m3_Angle)+*(this->base_robotCMD->yaw_speed)*robot_radius*(-1);

	for(int i=0;i<10;i++){
		if(fabs(w1_speed)>100||fabs(w2_speed)>100||fabs(w3_speed>100)){
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
	speed_regularization(w1_speed, w2_speed, w3_speed);
#ifdef DEBUG
	std::cout << "Inverse kinematics(DEBUG)\n";
	std::cout << std::dec;
	std::cout << "x_speed CMD: " << *(base_robotCMD->x_speed) << std::endl;
	std::cout << "y_speed CMD: " << *(base_robotCMD->y_speed) << std::endl;
	std::cout << "yaw_speed CMD: " << *(base_robotCMD->yaw_speed) << std::endl;
	std::cout << "w1_speed(%): " << w1_speed << std::endl;
	std::cout << "w2_speed(%): " << w2_speed << std::endl;
	std::cout << "w3_speed(%): " << w3_speed << std::endl;
	std::cout << std::endl;
#endif
}

void Base_Control::send(robot_command* CMD)
{
	this->base_robotCMD = CMD;
	shoot_regularization();
	inverseKinematics();
	//speed_regularization();
	mcssl_send2motor();
}

robot_command* Base_Control::get_feedback()
{
	forwardKinematics();
	return base_robotFB;
}
