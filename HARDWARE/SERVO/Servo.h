#ifndef _Servo_H
#define _Servo_H
#include "sys.h"



extern TIM_HandleTypeDef TIM2_Handler;      //定时器3PWM句柄 
extern TIM_OC_InitTypeDef TIM2_CH1Handler; 	//定时器2通道1句柄
extern TIM_OC_InitTypeDef TIM2_CH2Handler; 	//定时器2通道2句柄
extern TIM_OC_InitTypeDef TIM2_CH3Handler; 	//定时器2通道3句柄

void TIM2_Init(u16 arr,u16 psc);    		//定时器初始化
void TIM2_PWM_Init(u32 arr,u16 psc);
void TIM_SetTIM2Compare1(u32 compare);
void TIM_SetTIM2Compare2(u32 compare);
void TIM_SetTIM2Compare3(u32 compare);
void TIM_SetTIM2Compare4(u32 compare);
#endif

