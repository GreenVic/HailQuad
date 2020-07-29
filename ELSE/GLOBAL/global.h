#ifndef __GLOBAL__
#define __GLOBAL__


#include "sys.h"
//////////////////////////////Attitude.h////////////////////////////////
#define G 9.8f
#define DtoR 0.0175f //pi/180
#define SystemPeriod 0.0025f //50Hz，每个周期为0.020s
#define AttitudeFbP 0.5f
#define AttitudeFbI 0.05f
#define pi 3.1415f







//////////////////////////////Height.h////////////////////////////////////////////
#define VelocityHFbP 0.1f
#define VelocityHFbI 0.1f
#define HeightFbP 0.5f
#define HeightFbI 0.05f


//////////////////////////////Position.h///////////////////////////////////////////
#define PositionSNFbP 0.5f
#define PositionSNFbI 0.05f
#define PositionEWFbP 0.5f
#define PositionEWFbI 0.05f



//////////////////////////////Velocity.h////////////////////////////////////////////
#define LENGTH 50
#define VelocityEWFbP 0.5f
#define VelocityEWFbI 0.05f
#define VelocitySNFbP 0.5f
#define VelocitySNFbI 0.05f




////////////////////////////TakeOffLand.h////////////////////////////////////
#define CollectiveStage1  24370.0
#define CollectiveStage2  40640.0
#define TimeStage1 10.0
#define TimeStage2 1.0

#define TakeOffAcc  0.1
#define MaxHeightVel 0.5

#define LandAcc 0.1
#define MaxLandVel 2.0

#define OpenloopLandAcc 5
#define OpenloopMaxLandVel 5

#define BServoStatic2 124995
#define LServoStatic2 180087
#define RServoStatic2 124088
#define R_direction 1
#define B_direction 1
#define P_direction -1
#define T_direction 1
#define L_direction -1


////////////////////////////状态位////////////////////////////////////

extern u8 AutoStateFlag;


#endif
