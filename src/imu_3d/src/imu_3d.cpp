#include "ros/ros.h"
#include <freespace/freespace.h>
#include <freespace/freespace_util.h>
#include "appControlHandler.h"
#include "std_msgs/String.h"
#include "math.h"
#include "math/quaternion.h"
#include "math/vec3.h"
#include "imu_3d/inertia.h"
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/Twist.h"

#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>

#ifdef WIN32
#include <windows.h>

#include "win32/pthread_win32.h"
#define M_PI    3.141592654
#else
#include <unistd.h>
#include <pthread.h>
#endif

// Cross platform sleep macro
#ifdef _WIN32
#define SLEEP    Sleep(100)
#else
#define SLEEP    sleep(1)
#endif

#define RADIANS_TO_DEGREES(rad) ((float) rad * (float) (180.0 / M_PI))
#define DEGREES_TO_RADIANS(deg) ((float) deg * (float) (M_PI / 180.0))

using Eigen::Vector2d;

Vector2d vel;
Vector2d acc;
Vector2d gaga;
Vector2d vel_z;
Vector2d position;
double acc_mag;
double accel_x_filter;
double accel_y_filter;
double direct_tmp;
double degree;
double haha;
bool flag = true;
bool flag2 = true;
bool flag_move = true;
bool stationary = true;


//butterworth filter parameter

//for first-order
double BWA1 = 0;
double BWA2 = 0;
double BWA3 = 0;
double BWB1 = 0;
double BWB2 = 0;
double BWB3 = 0;
//for second-oder
/*double BWA1 = 1;
double BWA2 = -1.9112;
double BWA3 = 0.9150;
double BWB1 = 0.0009;
double BWB2 = 0.0019;
double BWB3 = 0.0009;*/

double HBW_X0 = 0;
double HBW_X1 = 0;
double HBW_X2 = 0;
double HBW_Y0 = 0;
double HBW_Y1 = 0;
double HBW_Y = 0;

double LBW_X0 = 0;
double LBW_X1 = 0;
double LBW_X2 = 0;
double LBW_Y0 = 0;
double LBW_Y1 = 0;
double LBW_Y = 0;

double ABW_X0 = 0;
double ABW_X1 = 0;
double ABW_X2 = 0;
double ABW_Y0 = 0;
double ABW_Y1 = 0;
double ABW_Y = 0;

double BBW_X0 = 0;
double BBW_X1 = 0;
double BBW_X2 = 0;
double BBW_Y0 = 0;
double BBW_Y1 = 0;
double BBW_Y = 0;

double BW_X0 = 0;
double BW_X1 = 0;
double BW_X2 = 0;
double BW_Y0 = 0;
double BW_Y1 = 0;
double BW_Y = 0;

int BW_COUNTER = 0;     //First-order
//int BW_COUNTER = 2;     //Second-order
//b =

//    0.0201    0.0402    0.0201
//a =

//    1.0000   -1.5610    0.6414

//Set high pass parameter
void setHP()
{
    BWA1 = 1;
    BWA2 = -0.9999;
    BWB1 = 1;
    BWB2 = -1;
    BW_COUNTER = 1;     //First-order
}
void setLP()
{
    BWA1 = 1;
    BWA2 = -0.7265;
    BWB1 = 0.1367;
    BWB2 = 0.1367;
    BW_COUNTER = 1;     //First-order
}

void setBW()
{
    BWA1 = 1;
    BWA2 = -0.9391;
    BWB1 = 0.0305;
    BWB2 = 0.0305;
    BW_COUNTER = 1;
}
void set2BW()
{
    BWA1 = 1;
    BWA2 = -1.5610;
    BWA3 = 0.6414;
    BWB1 = 0.0201;
    BWB2 = 0.0402;
    BWB3 = 0.0201;
    BW_COUNTER = 2;
}

/****************************************First-order of butterworth low-pass filter**********************************************/
double BW_SUB_first_order_Iteration(double X1, double X0, double Y0)
{
    double BW_OUT_TEMP = 0;
    BW_OUT_TEMP = BWB1*X1 + BWB2*X0 - BWA2*Y0;
    return BW_OUT_TEMP;
}

double BW_HIGH(double BW_X)
{
    if(BW_COUNTER == 0)
    {
        HBW_X1 = BW_X;
        HBW_Y = BW_SUB_first_order_Iteration(HBW_X1,HBW_X0,HBW_Y0);
    }
    if(BW_COUNTER ==1)
    {
        HBW_X0 = HBW_X1;
        HBW_X1 = BW_X;
        HBW_Y0 = HBW_Y;
        HBW_Y = BW_SUB_first_order_Iteration(HBW_X1,HBW_X0,HBW_Y0);
    }
    if(BW_COUNTER < 1)
    {
        BW_COUNTER++;
    }
    return HBW_Y*-1;        //Because of selecting format 1
}

double BW_LOW(double BW_X)
{
    if(BW_COUNTER == 0)
    {
        LBW_X1 = BW_X;
        LBW_Y = BW_SUB_first_order_Iteration(LBW_X1,LBW_X0,LBW_Y0);
    }
    if(BW_COUNTER ==1)
    {
        LBW_X0 = LBW_X1;
        LBW_X1 = BW_X;
        LBW_Y0 = LBW_Y;
        LBW_Y = BW_SUB_first_order_Iteration(LBW_X1,LBW_X0,LBW_Y0);
    }
    if(BW_COUNTER < 1)
    {
        BW_COUNTER++;
    }
    return LBW_Y*-1;        //Because of selecting format 1
}
double BW_Acc_x_first_order(double BW_X)
{
    if(BW_COUNTER == 0)
    {
        ABW_X1 = BW_X;
        ABW_Y = BW_SUB_first_order_Iteration(ABW_X1,ABW_X0,ABW_Y0);
    }
    if(BW_COUNTER ==1)
    {
        ABW_X0 = ABW_X1;
        ABW_X1 = BW_X;
        ABW_Y0 = ABW_Y;
        ABW_Y = BW_SUB_first_order_Iteration(ABW_X1,ABW_X0,ABW_Y0);
    }
    if(BW_COUNTER < 1)
    {
        BW_COUNTER++;
    }
    return ABW_Y;        //Because of selecting format 1
}
double BW_Acc_y_first_order(double BW_X)
{
    if(BW_COUNTER == 0)
    {
        BBW_X1 = BW_X;
        BBW_Y = BW_SUB_first_order_Iteration(BBW_X1,BBW_X0,BBW_Y0);
    }
    if(BW_COUNTER ==1)
    {
        BBW_X0 = BBW_X1;
        BBW_X1 = BW_X;
        BBW_Y0 = BBW_Y;
        BBW_Y = BW_SUB_first_order_Iteration(BBW_X1,BBW_X0,BBW_Y0);
    }
    if(BW_COUNTER < 1)
    {
        BW_COUNTER++;
    }
    return BBW_Y;        //Because of selecting format 1
}

double BW_SUB_Iteration(double X2, double X1, double X0, double Y1, double Y0){
    double BW_OUT_TEMP = 0;
    BW_OUT_TEMP = BWB1*X2 + BWB2*X1 + BWB3*X0 - BWA2*Y1 - BWA3*Y0;
    return BW_OUT_TEMP;
}

double BW_FILTER_second_order(double BW_X){
    if(BW_COUNTER == 0){
        BW_X2 = BW_X;
        BW_Y = BW_SUB_Iteration(BW_X2, BW_X1, BW_X0, BW_Y1, BW_Y0);
    }
    if(BW_COUNTER == 1){
        BW_X1 = BW_X2;
        BW_X2 = BW_X;
        BW_Y1 = BW_Y;
        BW_Y = BW_SUB_Iteration(BW_X2, BW_X1, BW_X0, BW_Y1, BW_Y0);
    }
    if(BW_COUNTER == 2){
        BW_X0 = BW_X1;
        BW_X1 = BW_X2;
        BW_X2 = BW_X;
        BW_Y0 = BW_Y1;
        BW_Y1 = BW_Y;
        BW_Y = BW_SUB_Iteration(BW_X2, BW_X1, BW_X0, BW_Y1, BW_Y0);
    }
    if(BW_COUNTER < 2){
        BW_COUNTER++;
    }
    return BW_Y;
}
/***************************************Second-order of butterworth low-pass filter***********************************************/
/*double BW_SUB_Iteration(double X2, double X1, double X0, double Y1, double Y0){
    double BW_OUT_TEMP = 0;
    BW_OUT_TEMP = BWB1*X2 + BWB2*X1 + BWB3*X0 - BWA2*Y1 - BWA3*Y0;
    return BW_OUT_TEMP;
}

double BW_FILTER_second_order(double BW_X){
    if(BW_COUNTER == 0){
        BW_X2 = BW_X;
        BW_Y = BW_SUB_Iteration(BW_X2, BW_X1, BW_X0, BW_Y1, BW_Y0);
    }
    if(BW_COUNTER == 1){
        BW_X1 = BW_X2;
        BW_X2 = BW_X;
        BW_Y1 = BW_Y;
        BW_Y = BW_SUB_Iteration(BW_X2, BW_X1, BW_X0, BW_Y1, BW_Y0);
    }
    if(BW_COUNTER == 2){
        BW_X0 = BW_X1;
        BW_X1 = BW_X2;
        BW_X2 = BW_X;
        BW_Y0 = BW_Y1;
        BW_Y1 = BW_Y;
        BW_Y = BW_SUB_Iteration(BW_X2, BW_X1, BW_X0, BW_Y1, BW_Y0);
    }
    if(BW_COUNTER < 2){
        BW_COUNTER++;
    }
    return BW_Y;
}*/

// State information for a thread
struct InputLoopState
{
    pthread_t thread_;     // A handle to the thread
    pthread_mutex_t lock_; // A mutex to allow access to shared data
    int quit_;             // An input to the thread

    // The shared data updated by the thread
    struct freespace_MotionEngineOutput meOut_; // Motion data
    int updated_; // A flag to indicate that the motion data has been updated
};

// ============================================================================
// Local function prototypes
// ============================================================================
static void* inputThreadFunction(void*);
static int getMotionFromInputThread(struct InputLoopState* state,
                                    struct freespace_MotionEngineOutput* meOut);
static void getEulerAnglesFromMotion(const struct freespace_MotionEngineOutput* meOut,
                                     struct Vec3f* eulerAngles);

// ============================================================================
// Public functions
// ============================================================================

/******************************************************************************
 * main
 * This example uses the synchronous API to access the device.
 * It assumes the device is already connected. It uses pthreads to put the
 * operation of handling incoming messages from the device in a separate thread
 * from the main thread. It configures the device to produce fused motion outputs.
 */

/**********************Rotate body accelerations to Earth frame**********************/
void rotate(double a_y, double a_x, double angle)
{
    acc(0) = (a_x*cos(angle) + a_y*cos(angle+(M_PI/2)))*9.8;    //Plus 9.8 because of selecting format 1
    acc(1) = (a_x*sin(angle) + a_y*sin(angle+(M_PI/2)))*9.8;    //Plus 9.8 because of selecting format 1
}

/**********************Calculate velocity**********************/
void calc_vel(Vector2d a,double t)
{
    vel = vel_z + a*t;
    //printf("V0_x:%f\tV0_y:%f\n",vel_z(0),vel_z(1));
}

/**********************Calculate shift**********************/
void calc_shift(Vector2d a,double t)
{
    //position = vel_z*t + a*t*t/2 + position;
    position = vel_z*t + a*t*t/2;
    //position = position + vel*t;
}

void speed_stationary(const geometry_msgs::Twist::ConstPtr& speed)
{
    gaga(0) = speed->linear.x;
    gaga(1) = speed->linear.y;
    haha = sqrt(gaga(0)*gaga(0) + gaga(1)*gaga(1));
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "imu_3d");
    ros::NodeHandle n;
    ros::Publisher imu_pub = n.advertise<imu_3d::inertia>("imu_3d",1000);
    ros::Subscriber speed_sub = n.subscribe("/FIRA/R1/Strategy/PathPlan/RobotSpeed",100,speed_stationary);

    ros::Rate loop_rate(100);

    struct InputLoopState inputLoop;
    struct freespace_MotionEngineOutput meOut;
    struct Vec3f eulerAngles;
    struct MultiAxisSensor accel;
    struct MultiAxisSensor sensor;
    struct MultiAxisSensor angularVel;
    int rc;

    // Flag to indicate that the application should quit
    // Set by the control signal handler
    int quit = 0;
    printVersionInfo(argv[0]);
    addControlHandler(&quit);

    // Initialize the freespace library
    rc = freespace_init();
    if (rc != FREESPACE_SUCCESS) {
        printf("Initialization error. rc=%d\n", rc);
        exit(1);
    }

    // Setup the input loop thread
    memset(&inputLoop, 0, sizeof(struct InputLoopState));                      // Clear the state info for the thread
    pthread_mutex_init(&inputLoop.lock_, NULL);                                // Initialize the mutex
    pthread_create(&inputLoop.thread_, NULL, inputThreadFunction, &inputLoop); // Start the input thread
    while(!quit)
    {
        // Get input.
        rc = getMotionFromInputThread(&inputLoop, &meOut);

        // If new motion was available, use it
        if (rc) {
            int i;
            // Run game logic.
            getEulerAnglesFromMotion(&meOut, &eulerAngles);
            freespace_util_getCompassHeading(&meOut, &sensor);
            freespace_util_getAccNoGravity(&meOut, &accel);
            freespace_util_getAngularVelocity(&meOut, &angularVel);
            //freespace_util_getAcceleration(&meOut, &accel);

            //double gaga_tmp = sqrt(angularVel.z*angularVel.z);

            acc_mag = sqrt(accel.x*accel.x + accel.y*accel.y);
            setHP();
            double acc_filter = BW_HIGH(acc_mag);
            if(acc_filter > 0)
                acc_filter = abs(acc_filter);

            setLP();
            acc_filter = BW_LOW(acc_filter);

            //setBW();
            accel_x_filter = BW_Acc_x_first_order(accel.x);
            accel_y_filter = BW_Acc_y_first_order(accel.y);
            if(accel_x_filter<0)
                accel_x_filter = accel_x_filter*-1;
            if(accel_y_filter<0)
                accel_y_filter = accel_y_filter*-1;

            //printf("%f,",accel_x_filter);

            //setHP();
            //double gaga = BW_Acc_x_first_order(gaga_tmp);
//            set2BW();
//            gaga = BW_FILTER_second_order(gaga_tmp);
//            printf("gaga=%f\t",gaga);

//            if(gaga<0.014)
//            {
//                stationary = true;
//                printf("1\n");
//            }else{
//                stationary = false;
//                printf("0\n");
//            }
            if(haha!=0)
            {
                stationary = false;
                printf("0\n");
            }else{
                stationary = true;
                printf("1\n");
            }

            //Render
            //printf("%f\n",accel.y);
            //printf("acc_mag:%f\n",acc_mag);
            //printf("roll: %0.4f, pitch: %0.4f, yaw: %0.4f\n",RADIANS_TO_DEGREES(eulerAngles.x),RADIANS_TO_DEGREES(eulerAngles.y),RADIANS_TO_DEGREES(eulerAngles.z));
            //printf("accel\tx: %0.4f, y: %0.4f, z: %0.4f\n",accel.x,accel.y,accel.z);
            fflush(stdout);

            //Calculate sample time
            double t;
            double last_time;
            double delta_t = 0;
            degree = 360+sensor.x-direct_tmp;
            if(degree>=360)
                degree = degree - 360;
            //printf("degree:%f\n",degree);
            //printf("sensor:%f\n",sensor.x);
            i++;

            t = ros::Time::now().toSec();
            if(flag)
            {
                delta_t = 0;
                direct_tmp = sensor.x;
                flag = false;
            }else{
                delta_t = t - last_time;
                //printf("%f\n",delta_t);
                flag2 = false;
            }
            last_time = t;

            //Calculate shift
            rotate(-(accel.x-0.0048),-(accel.y-0.0051),DEGREES_TO_RADIANS(degree));     //原訊號
            //rotate(accel_x,accel_y,DEGREES_TO_RADIANS(degree));

            if(stationary)
            {
                vel(0) = 0;
                vel(1) = 0;
            }else{
                if(accel_x_filter>0.01 && accel_y_filter>0.01)
                {
                    calc_vel(acc,delta_t);
                }else if(accel_x_filter>0.01 && accel_y_filter<0.01)
                {
                    acc(0) = 0;
                    calc_vel(acc,delta_t);
                }else if(accel_x_filter<0.01 && accel_y_filter>0.01)
                {
                    acc(1) = 0;
                    calc_vel(acc,delta_t);
                }else{
                    vel = vel_z;
                    acc(0)=0;
                    acc(1)=0;
                }
            }

            if(sqrt(vel(0)*vel(0)+vel(1)*vel(1))>1.5)
            {
                acc(0) = 0;
                acc(1) = 0;
            }

            if(flag2)
            {
            }else{
                calc_shift(acc,delta_t);
            }
            vel_z = vel;
        }

        //ROS publish msg
        imu_3d::inertia inertia;

        inertia.shift_x = position(0);
        inertia.shift_y = position(1);

        //printf("%f\t%f\n",position(0),position(1));

        inertia.yaw = DEGREES_TO_RADIANS(degree);

        imu_pub.publish(inertia);

        ros::spinOnce();
        loop_rate.sleep();
    }
    // Cleanup the input loop thread
    inputLoop.quit_ = 1;                     // Signal the thread to stop
    pthread_join(inputLoop.thread_, NULL);   // Wait until it does
    pthread_mutex_destroy(&inputLoop.lock_); // Get rid of the mutex

    // Finish using the library gracefully
    freespace_exit();
    return 0;
}
// ============================================================================
// Local functions
// ============================================================================

/******************************************************************************
 * getMotionFromInputThread
 *
 * @param state a pointer the the shared state information for the input loop thread
 * @param meOut a pointer to where to copy the motin information retrieved from the inpuit loop thread
 * @param return the updated flag from the input loop thread state
 */
static int getMotionFromInputThread(struct InputLoopState * state,struct freespace_MotionEngineOutput * meOut)
{
    int updated;

    pthread_mutex_lock(&state->lock_);   // Obtain ownership of the input loop thread's shared state information
    *meOut = state->meOut_;              // Copy the motion packet to the main thread
    updated = state->updated_;           // Remember the updated_ flag
    state->updated_ = 0;                 // Mark the data as read
    pthread_mutex_unlock(&state->lock_); // Release ownership of the input loop thread's shared state information

    return updated;
}


/******************************************************************************
 * getEulerAnglesFromMotion
 */
static void getEulerAnglesFromMotion(const struct freespace_MotionEngineOutput* meOut,struct Vec3f* eulerAngles)
{
    struct MultiAxisSensor sensor;
    struct Quaternion q;

    // Get the angular position data from the MEOut packet
    freespace_util_getAngPos(meOut, &sensor);

    // Copy the data over to because both the util API and quaternion.h each have their own structs
    q.w = sensor.w;
    q.x = sensor.x;
    q.y = sensor.y;
    q.z = sensor.z;

    // The Freespace quaternion gives the rotation in terms of
    // rotating the world around the object. We take the conjugate to
    // get the rotation in the object's reference frame.
    q_conjugate(&q, &q);

    // Convert quaternion to Euler angles
    q_toEulerAngles(eulerAngles, &q);
}

// ============================================================================
// Thread functions
// ============================================================================

/******************************************************************************
 * inputThreadFunction
 */
static void* inputThreadFunction(void* arg)
{
    struct InputLoopState* state = (struct InputLoopState*) arg;
    struct freespace_message message;
    FreespaceDeviceId device;
    int numIds;
    int rc;

    /** --- START EXAMPLE INITIALIZATION OF DEVICE -- **/
    // This example requires that the freespace device already be connected
    // to the system before launching the example.
    rc = freespace_getDeviceList(&device, 1, &numIds);
    if (numIds == 0) {
        printf("freespaceInputThread: Didn't find any devices.\n");
        exit(1);
    }

    rc = freespace_openDevice(device);
    if (rc != FREESPACE_SUCCESS) {
        printf("freespaceInputThread: Error opening device: %d\n", rc);
        exit(1);
    }

    rc = freespace_flush(device);
    if (rc != FREESPACE_SUCCESS) {
        printf("freespaceInputThread: Error flushing device: %d\n", rc);
        exit(1);
    }

    // Put the device in the right operating mode
    memset(&message, 0, sizeof(message));
    message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
    message.dataModeControlV2Request.packetSelect = 8; // MEOut
    message.dataModeControlV2Request.mode = 4;         // Set full motion - ALWAYS ON

    //message.dataModeControlV2Request.formatSelect = 0; // MEOut format set 0 for accel with g
    message.dataModeControlV2Request.formatSelect = 1; // MEOut format set 1 for accel without g
    message.dataModeControlV2Request.ff0 = 1;          // Acceleration fields
    message.dataModeControlV2Request.ff1 = 1;
    message.dataModeControlV2Request.ff2 = 1;
    message.dataModeControlV2Request.ff3 = 1;
    message.dataModeControlV2Request.ff4 = 1;          // Acceleration fields
    message.dataModeControlV2Request.ff5 = 1;
    message.dataModeControlV2Request.ff6 = 1;          // Angular (orientation) fields

    rc = freespace_sendMessage(device, &message);
    if (rc != FREESPACE_SUCCESS) {
        printf("freespaceInputThread: Could not send message: %d.\n", rc);
    }
    /** --- END EXAMPLE INITIALIZATION OF DEVICE -- **/

    // The input loop
    while (!state->quit_) {
        rc = freespace_readMessage(device, &message, 1000 /* 1 second timeout */);
        if (rc == FREESPACE_ERROR_TIMEOUT ||
            rc == FREESPACE_ERROR_INTERRUPTED) {
            continue;
        }
        if (rc != FREESPACE_SUCCESS) {
            printf("freespaceInputThread: Error reading: %d. Trying again after a second...\n", rc);
            SLEEP;
            continue;
        }

        // Check if this is a MEOut message.
        if (message.messageType == FREESPACE_MESSAGE_MOTIONENGINEOUTPUT) {
            pthread_mutex_lock(&state->lock_);

            // Update state fields.
            state->meOut_ = message.motionEngineOutput;
            state->updated_ = 1;

            pthread_mutex_unlock(&state->lock_);
        }
    }

    /** --- START EXAMPLE FINALIZATION OF DEVICE --- **/
    printf("\n\nfreespaceInputThread: Cleaning up...\n");
    memset(&message, 0, sizeof(message));
    message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
    message.dataModeControlV2Request.packetSelect = 1;  // Mouse packets
    message.dataModeControlV2Request.mode = 0;          // Set full motion
    rc = freespace_sendMessage(device, &message);
    if (rc != FREESPACE_SUCCESS) {
        printf("freespaceInputThread: Could not send message: %d.\n", rc);
    }

    //freespace_closeDevice(device);
    /** --- END EXAMPLE FINALIZATION OF DEVICE --- **/

    // Exit the thread.
    return 0;
}
