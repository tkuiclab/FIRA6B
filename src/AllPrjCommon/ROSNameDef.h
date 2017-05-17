#ifndef ROS_NAME_DEF_H_
#define ROS_NAME_DEF_H_




//======================================Vision======================================//
#define Vision_Segmentation_Topic               "/FIRA/Vision/Segmentation"
#define Vision_BlackRealDis_Topic               "/FIRA/Vision/BlackRealDis"

#define blackitem_range                         90


//======================================Strategy======================================//
#define R1_Strategy_Coach_Role_Topic            "/FIRA/R1/Strategy/Coach/Role"
#define R2_Strategy_Coach_Role_Topic            "/FIRA/R2/Strategy/Coach/Role"
#define R3_Strategy_Coach_Role_Topic            "/FIRA/R3/Strategy/Coach/Role"

#define R1_Strategy_PathPlan_RobotSpeed_Topic   "/FIRA/R1/Strategy/PathPlan/RobotSpeed"
#define R2_Strategy_PathPlan_RobotSpeed_Topic   "/FIRA/R2/Strategy/PathPlan/RobotSpeed"
#define R3_Strategy_PathPlan_RobotSpeed_Topic   "/FIRA/R3/Strategy/PathPlan/RobotSpeed"

#define R1_Strategy_WorldMap_RobotPos_Topic     "/FIRA/R1/Strategy/WorldMap/RobotPos"
#define R2_Strategy_WorldMap_RobotPos_Topic     "/FIRA/R2/Strategy/WorldMap/RobotPos"
#define R3_Strategy_WorldMap_RobotPos_Topic     "/FIRA/R3/Strategy/WorldMap/RobotPos"

#define Strategy_WorldMap_Soccer_Topic          "/FIRA/Strategy/WorldMap/soccer"

//Challenge Topic
#define Strategy_Challenge_Location_Topic       "/FIRA/Strategy/Challenge/Location"
#define Strategy_Challenge_AvoidBarriers_Topic  "/FIRA/Strategy/Challenge/AvoidBarriers"

//GameState Srv
#define Strategy_GameState_Srv                  "/FIRA/Strategy/GameState"

//Link
#define R1_FPGALink_Topic                       "/FIRA/R1/FPGALink"
#define R2_FPGALink_Topic                       "/FIRA/R2/FPGALink"
#define R3_FPGALink_Topic                       "/FIRA/R3/FPGALink"

#define R1_IPCAMLink_Topic                      "/FIRA/R1/IPCAMLink"
#define R2_IPCAMLink_Topic                      "/FIRA/R2/IPCAMLink"
#define R3_IPCAMLink_Topic                      "/FIRA/R3/IPCAMLink"

//Simulator
#define Simulator_Enable_Topic                  "/FIRA/Simulator_Enable"

#define white_ctrl                              "white_ctrl.jpg"
#define red_jpg                                 "red.jpg"
#define green_jpg                               "green.jpg"

//======================================System======================================//
#define SpeedMode_MS      1      //unit m/s
#define SpeedMode_Range   2      //unit range 0~100

#define SpeedMode_Range_Size          100
#define SpeedMode_Range_MaxSpeed      1.5
#define SpeedMode_Range_MaxRotation   2*M_PI

#endif
