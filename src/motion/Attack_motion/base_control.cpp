#include "base_control.h"

//#include "../common/cssl/cssl.h"
#include "cssl.c"
#include "port.h"


Base_Control::Base_Control()
{
	this->base_TX = new serial_tx;

	this->base_TX->head1 = new unsigned char;
	this->base_TX->head2 = new unsigned char;
	this->base_TX->w1 = new unsigned char;
	this->base_TX->w2 = new unsigned char;
	this->base_TX->w3 = new unsigned char;
	this->base_TX->enable_and_stop = new unsigned char;
	this->base_TX->shoot = new unsigned char;
	this->base_TX->checksum = new unsigned char;

	memset(this->base_TX->head1, 0xff, sizeof(unsigned char));
	memset(this->base_TX->head2, 0xfa, sizeof(unsigned char));
	memset(this->base_TX->w1, 0, sizeof(unsigned char));
	memset(this->base_TX->w2, 0, sizeof(unsigned char));
	memset(this->base_TX->w3, 0, sizeof(unsigned char));
	memset(this->base_TX->enable_and_stop, 0, sizeof(unsigned char));
	memset(this->base_TX->shoot, 0, sizeof(unsigned char));
	memset(this->base_TX->checksum, 0, sizeof(unsigned char));
	//this->base_RX = new serial_rx;
	this->w1_speed=0;this->w1_speed_percent=0;
	this->w2_speed=0;this->w2_speed_percent=0;
	this->w3_speed=0;this->w3_speed_percent=0;
	this->en1=0;this->en2=0;this->en3=0;
	this->stop1=0;this->stop2=0;this->stop3=0;
	this->w1_dir=0;this->w2_dir=0;this->w3_dir=0;
	this->serial = NULL;
#ifdef DEBUG
	std::cout << "Base_Control(DEBUG)\n";
	std::cout << "Init base control\n";
	std::cout << (int)*this->base_TX->head1 << std::endl;
	std::cout << (int)*this->base_TX->head2 << std::endl;
	std::cout << (int)*this->base_TX->w1 << std::endl;
	std::cout << (int)*this->base_TX->w2 << std::endl;
	std::cout << (int)*this->base_TX->w3 << std::endl;
	std::cout << (int)*this->base_TX->enable_and_stop << std::endl;
	std::cout << (int)*this->base_TX->shoot << std::endl;
	std::cout << (int)*this->base_TX->checksum << std::endl;
#endif
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
/*	static int i=0;
    int checknum;
    unsigned char pls1_1,pls1_2,pls1_3,pls1_4;
    unsigned char pls2_1,pls2_2,pls2_3,pls2_4;
    unsigned char pls3_1,pls3_2,pls3_3,pls3_4;
    bool findpacket=0;
//        printf("*buf=%x\n",*buf);
    //-----store packet
    feedback[i]=*buf;

//    printf("feedback[%d]=%d\n",i,feedback[i]);
    i=i+length;
    if((i>14))i=i%15;

//    printf("feedback:\n");
//    for(int oo=0;oo<15;oo++){
//        printf("%x ,",feedback[oo]);
//    }
//    printf("\n");

    for(int header=0;header<15;header++){
//                printf("feedback:\n");
//                for(int oo=0;oo<15;oo++){
//                    printf("%x ,",feedback[(header+oo)%15]);
//                }
//                printf("\n");
        if(feedback[header]!=0xff){continue;}
        else if(feedback[(header+1)%15]!=0xfa){continue;}
        else{
            if(header == 0){checknum=15;}
            else{checknum=header;}
            unsigned char checkcode = feedback[checknum-1];
            unsigned char checksum = 0;
            for(int check=2;check<14;check++){
                checksum += feedback[(header+check)%15];
            }
//                printf("checksum = %x checknum = %x\n",checksum,checkcode);
            if(checksum != checkcode){continue;}
            else{
                findpacket=1;
//                    printf("checksum = %x checknum = %x\n",checksum,checkcode);
//                    printf("feedback[14]=%x ,checkcode=%x\n",feedback[14],checkcode);
                if(feedback[14]!=checkcode){
//                    printf("checksum = %x checknum = %x\n",checksum,checkcode);
	                for(int permutation=15;permutation<15-header;permutation--){
	                    unsigned char tmp[2];
//                        printf("move:\n");
	                    for(int move=0;move<15;move++){
	                        tmp[1] = feedback[(header+move+1)%15];
	                        tmp[0] = tmp[1];
	                        feedback[(header+move+1)%15] = tmp[0];
//                            printf("%x ,",feedback[move]);
	                    }
//                        printf("\n");
	                }
	                i = 0;
                }else{break;}
            }
        }
    }
    
//    static int aa=0;
//    static bool bb = 0;
//    static double  start;
//    if(bb == 0){
//        start = ros::Time::now().toSec();
//        bb = 1;
//    }
//    double now = ros::Time::now().toSec();
//    printf("[Feedback start time:%f]\n",now-start);
//    printf("%d ",aa++);
//    printf("feedback:\n");
//    for(int oo=0;oo<15;oo++){
//        printf("%x ,",feedback[oo]);
//    }
//    printf("findpacket %d\n",findpacket);
    if((findpacket==1)&&(feedback[0]==0xff)&&(feedback[1]==0xfa)){
//            printf("feedback:\n");
//            for(int oo=0;oo<15;oo++){
//                printf("%x ,",feedback[oo]);
//            }
//            printf("\n");
        pls1_4 = feedback[2];   //motor1
        pls1_3 = feedback[3];
        pls1_2 = feedback[4];
        pls1_1 = feedback[5];

        pls2_4 = feedback[6];   //motor2
        pls2_3 = feedback[7];
        pls2_2 = feedback[8];
        pls2_1 = feedback[9];

        pls3_4 = feedback[10];  //motor3
        pls3_3 = feedback[11];
        pls3_2 = feedback[12];
        pls3_1 = feedback[13];
        *(int*)base_motorFB->w1_speed = (pls1_4 << 24)+(pls1_3 << 16)+(pls1_2 << 8)+pls1_1;
        *(int*)base_motorFB->w2_speed = (pls2_4 << 24)+(pls2_3 << 16)+(pls2_2 << 8)+pls2_1;
        *(int*)base_motorFB->w3_speed = (pls3_4 << 24)+(pls3_3 << 16)+(pls3_2 << 8)+pls3_1;
    }else{
		base_motorFB = base_motorFB;
    }



//    printf("pls_1=%d \n",pls3);
    odom_x =(pls1*(-0.3333)+pls2*(-0.3333)+pls3*(0.6667))*2*M_PI*WheelRadius/(26)/2000;
    odom_y =(pls1*(0.5774) + pls2*(-0.5774) + pls3*(0))*2*M_PI*WheelRadius/26/2000;
//    odom_yaw =(pls1*(0.21231) + pls2*(0.21231) + pls3*(0.21231))/2000/26;
    odom_yaw =(pls1*(yaw_inv) + pls2*(yaw_inv) + pls3*(yaw_inv))*2*M_PI*WheelRadius/2000/26;
    int round;
    round = odom_yaw/(2*M_PI);
    odom_yaw = (odom_yaw-round*2*M_PI);
//    printf("round = %d\n");
//    printf("length_x=%lf , length_y=%lf , length_r=%lf\npls_1=%d,pls_2=%d, pls_3=%d \n\n",odom_x,odom_y,odom_yaw,pls1,pls2,pls3);
//    printf("\n=====================\n");

    motionfeedback.linear.x = odom_x;
    motionfeedback.linear.y = odom_y;
    motionfeedback.angular.z = odom_yaw;
 */
#ifdef DEBUG_CSSLCALLBACK
	std::cout << "mcssl_Callback(DEBUG_CSSLCALLBACK)\n";
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
	std::cout << (int)stop2 << (int)stop3 << "00 (" << (int)*(this->base_TX->enable_and->stop) << ")"<<  std::endl;
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

void Base_Control::inverseKinematics()
{
	this->w1_speed = *(this->base_robotCMD->x_speed)*cos(m1_Angle)+*(this->base_robotCMD->y_speed)*sin(m1_Angle)+*(this->base_robotCMD->yaw_speed)*robot_radius*(-1);
	this->w2_speed = *(this->base_robotCMD->x_speed)*cos(m2_Angle)+*(this->base_robotCMD->y_speed)*sin(m2_Angle)+*(this->base_robotCMD->yaw_speed)*robot_radius*(-1);
	this->w3_speed = *(this->base_robotCMD->x_speed)*cos(m3_Angle)+*(this->base_robotCMD->y_speed)*sin(m3_Angle)+*(this->base_robotCMD->yaw_speed)*robot_radius*(-1);

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
	std::cout << "x_speed CMD: " << *(base_robotCMD->x_speed) << std::endl;
	std::cout << "y_speed CMD: " << *(base_robotCMD->y_speed) << std::endl;
	std::cout << "yaw_speed CMD: " << *(base_robotCMD->yaw_speed) << std::endl;
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

