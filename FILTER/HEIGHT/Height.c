#ifndef __HEIGHT_H
#define __HEIGHT_H


#include "Height.h"



float VelocityH[3];//垂直方向上的速度
float Height[3];//融合滤波后的高度
float GPSHeight[3];//GPS得到的高度


void heightFilter(void)
{
	
	GPSHeight[2] = GPSHeight[1];
	GPSHeight[1] = GPSHeight[0]; 
	GPSHeight[0] = gps.altitude/1000.0;     //数据转换
	//加速度滤波得到垂直方向上的速度
	VelocityH[2] = VelocityH[1];
	VelocityH[1] = VelocityH[0]; 
	
	VelocityH[0] = 2 * (EarthAcc[0][2]*G-G) * SystemPeriod - 2 * (EarthAcc[2][2]*G-G) * SystemPeriod;
   // printf("%f,",GPSHeight[0]);
	VelocityH[0] = (VelocityH[0] - (2*VelocityHFbI*SystemPeriod*SystemPeriod-8) * VelocityH[1] - \
	(VelocityHFbI*SystemPeriod*SystemPeriod-2*VelocityHFbP*SystemPeriod+4) * VelocityH[2])/(VelocityHFbI*SystemPeriod*SystemPeriod+2*VelocityHFbP*SystemPeriod+4);
	//printf("%f;\r\n",Height[0]);
	//垂直方向上的速度与GPS得到的高度融合滤波得到高度
	Height[2] = Height[1];
	Height[1] = Height[0];
	Height[0] = 2 * VelocityH[0] * SystemPeriod - 2 * VelocityH[2] * SystemPeriod + \
	(HeightFbI*SystemPeriod*SystemPeriod+2*HeightFbP*SystemPeriod)* GPSHeight[0] + \
	(HeightFbI*SystemPeriod*SystemPeriod-2*HeightFbP*SystemPeriod) * GPSHeight[2] + 2*HeightFbI*SystemPeriod*SystemPeriod * GPSHeight[1];
    
	Height[0] = (Height[0] - (2*HeightFbI*SystemPeriod*SystemPeriod-8) * Height[1] - \
	(HeightFbI*SystemPeriod*SystemPeriod-2*HeightFbP*SystemPeriod+4) * Height[2])/(HeightFbI*SystemPeriod*SystemPeriod+2*HeightFbP*SystemPeriod+4);
}

#endif

