#ifndef _Servo_H
#define _Servo_H
#include "sys.h"



extern TIM_HandleTypeDef TIM2_Handler;      //��ʱ��3PWM��� 
extern TIM_OC_InitTypeDef TIM2_CH1Handler; 	//��ʱ��2ͨ��1���
extern TIM_OC_InitTypeDef TIM2_CH2Handler; 	//��ʱ��2ͨ��2���
extern TIM_OC_InitTypeDef TIM2_CH3Handler; 	//��ʱ��2ͨ��3���

void TIM2_Init(u16 arr,u16 psc);    		//��ʱ����ʼ��
void TIM2_PWM_Init(u32 arr,u16 psc);
void TIM_SetTIM2Compare1(u32 compare);
void TIM_SetTIM2Compare2(u32 compare);
void TIM_SetTIM2Compare3(u32 compare);
void TIM_SetTIM2Compare4(u32 compare);
#endif

