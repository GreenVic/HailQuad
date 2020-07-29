#ifndef _HEIGHT_H
#define	_HEIGHT_H
#include "gps.h"
#include "global.h"
#include "Attitude.h"



extern float VelocityH[3];//垂直方向上的速度
extern float Height[3];//融合滤波后的高度
extern float GPSHeight[3];//GPS得到的高度



void heightFilter(void);
#endif
