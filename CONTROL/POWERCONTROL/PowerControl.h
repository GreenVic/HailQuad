#ifndef _POWERCONTROL__H
#define _POWERCONTROL__H

#include "Attitude.h"
#include "AttitudeControl.h"
#define Ratio 2

//�ɿظ����Ķ������
extern unsigned int MOTOR_M1;//���ϨJ
extern unsigned int MOTOR_M2;//���¨L
extern unsigned int MOTOR_M3;//���ϨI
extern unsigned int MOTOR_M4;//���¨K

extern s16 ControlHeight;



void ReceiveOrder(void);            //����ң����ָ��
void Motor_Set(void);                     //�������
void Motor_PWMSet( unsigned int m1_set,unsigned int m2_set, unsigned int m3_set, unsigned int m4_set);    //PWM�������
















#endif
