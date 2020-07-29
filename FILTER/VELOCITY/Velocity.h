#ifndef _VELOCITY_H
#define	_VELOCITY_H

#include "gps.h"
#include "Attitude.h"
#include "math.h"
#include "global.h"


extern float VelocitySN[3],VelocityEW[3];
void VelocityFilter(nmea_msg gps);


extern float Body_AccLength,Body_AccCross; //机身纵向\横向水平加速度
extern	float Filter_AccLength1,Filter_AccCross1,Filter_AccLength2,Filter_AccCross2;     //通过一阶二阶滤波的机体加速度
extern	float Earth_AccSN[3],Earth_AccEW[3];    //地面坐标系的加速度
extern	float Filter_AccSN1,Filter_AccEW1,Filter_AccSN2,Filter_AccEW2;     //通过一阶二阶滤波的地面加速度
extern	float AccFilterParam;
	
	
	
extern	float GPS_Yaw,Gps_Velocity;
extern	float GPS_VelSN,GPS_VelEW;
extern	float Earth_VelSN[3],Earth_VelEW[3];

#endif
