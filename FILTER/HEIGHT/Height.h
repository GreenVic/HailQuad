#ifndef _HEIGHT_H
#define	_HEIGHT_H
#include "gps.h"
#include "global.h"
#include "Attitude.h"



extern float VelocityH[3];//��ֱ�����ϵ��ٶ�
extern float Height[3];//�ں��˲���ĸ߶�
extern float GPSHeight[3];//GPS�õ��ĸ߶�



void heightFilter(void);
#endif
