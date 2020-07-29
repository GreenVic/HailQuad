#ifndef _POWERCONTROL__H
#define _POWERCONTROL__H

#include "Attitude.h"
#include "AttitudeControl.h"
#define Ratio 2

//飞控给出的舵机输入
extern unsigned int MOTOR_M1;//右上↗
extern unsigned int MOTOR_M2;//左下↙
extern unsigned int MOTOR_M3;//左上↖
extern unsigned int MOTOR_M4;//右下↘

extern s16 ControlHeight;



void ReceiveOrder(void);            //接受遥控器指令
void Motor_Set(void);                     //电机分配
void Motor_PWMSet( unsigned int m1_set,unsigned int m2_set, unsigned int m3_set, unsigned int m4_set);    //PWM电机调速
















#endif
