#include "Servo.h"



TIM_HandleTypeDef TIM2_Handler;      	//��ʱ����� 
TIM_OC_InitTypeDef TIM2_CH1Handler;     //��ʱ��2ͨ��1���
TIM_OC_InitTypeDef TIM2_CH2Handler;     //��ʱ��2ͨ��2���
TIM_OC_InitTypeDef TIM2_CH3Handler;     //��ʱ��2ͨ��3���
TIM_OC_InitTypeDef TIM2_CH4Handler;     //��ʱ��2ͨ��4���
//ͨ�ö�ʱ��3�жϳ�ʼ��,��ʱ��3��APB1�ϣ�APB1�Ķ�ʱ��ʱ��Ϊ200MHz
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!(��ʱ��3����APB1�ϣ�ʱ��ΪHCLK/2)
void TIM2_Init(u16 arr,u16 psc)
{  
    TIM2_Handler.Instance=TIM2;                          //ͨ�ö�ʱ��3
    TIM2_Handler.Init.Prescaler=psc;                     //��Ƶ
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM2_Handler.Init.Period=arr;                        //�Զ�װ��ֵ
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//ʱ�ӷ�Ƶ����
    HAL_TIM_Base_Init(&TIM2_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM2_Handler); //ʹ�ܶ�ʱ��3�Ͷ�ʱ��3�����жϣ�TIM_IT_UPDATE    
}

//TIM3 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM2_PWM_Init(u32 arr,u16 psc)
{ 
	 TIM2_Handler.Instance=TIM2;            //��ʱ��3
    TIM2_Handler.Init.Prescaler=psc;       //��ʱ����Ƶ
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//���ϼ���ģʽ
    TIM2_Handler.Init.Period=arr;          //�Զ���װ��ֵ
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM2_Handler);       //��ʼ��PWM
    //ͨ��1
    TIM2_CH1Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM2_CH1Handler.Pulse=10000;            //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�
                                            //Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM2_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //����Ƚϼ���Ϊ�� 
	TIM2_CH1Handler.OCFastMode=TIM_OCFAST_ENABLE;
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handler,&TIM2_CH1Handler,TIM_CHANNEL_1);//����TIM2ͨ��1
    HAL_TIM_PWM_Start(&TIM2_Handler,TIM_CHANNEL_1);//����PWMͨ��1
	
	//ͨ��2
	TIM2_CH2Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM2_CH2Handler.Pulse=10000;            //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�
                                            //Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM2_CH2Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //����Ƚϼ���Ϊ�� 
	TIM2_CH2Handler.OCFastMode=TIM_OCFAST_ENABLE;
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handler,&TIM2_CH2Handler,TIM_CHANNEL_2);//����TIM2ͨ��2
    HAL_TIM_PWM_Start(&TIM2_Handler,TIM_CHANNEL_2);//����PWMͨ��2
	//ͨ��3
	TIM2_CH3Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM2_CH3Handler.Pulse=10000;            //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�
                                            //Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM2_CH3Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //����Ƚϼ���Ϊ�� 
	TIM2_CH3Handler.OCFastMode=TIM_OCFAST_ENABLE;
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handler,&TIM2_CH3Handler,TIM_CHANNEL_3);//����TIM2ͨ��3
    HAL_TIM_PWM_Start(&TIM2_Handler,TIM_CHANNEL_3);//����PWMͨ��3
	
	//ͨ��4
	TIM2_CH4Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM2_CH4Handler.Pulse=10000;            //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�
                                            //Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM2_CH4Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //����Ƚϼ���Ϊ�� 
	TIM2_CH4Handler.OCFastMode=TIM_OCFAST_ENABLE;
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handler,&TIM2_CH4Handler,TIM_CHANNEL_4);//����TIM2ͨ��2
    HAL_TIM_PWM_Start(&TIM2_Handler,TIM_CHANNEL_4);//����PWMͨ��4
}

//��ʱ���ײ�������ʱ��ʹ�ܣ���������
//�˺����ᱻHAL_TIM_PWM_Init()����
//htim:��ʱ�����
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
   //ͨ��1
    GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_TIM2_CLK_ENABLE();			//ʹ�ܶ�ʱ��2
    __HAL_RCC_GPIOA_CLK_ENABLE();			//����GPIOAʱ��
	
    GPIO_Initure.Pin=GPIO_PIN_5;           	//PA5
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//�����������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //����
	GPIO_Initure.Alternate=GPIO_AF1_TIM2;	//PB1����ΪTIM3_CH4
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	//ͨ��2
	
    GPIO_Initure.Pin=GPIO_PIN_1;           	//PA1
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	//ͨ��3
	
    GPIO_Initure.Pin=GPIO_PIN_2;           	//PA2
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	
	//ͨ��4
	
    GPIO_Initure.Pin=GPIO_PIN_3;           	//PA3
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
}

//��ʱ���ײ�����������ʱ�ӣ������ж����ȼ�
//�˺����ᱻHAL_TIM_Base_Init()��������
//void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
//{
//    if(htim->Instance==TIM2)
//	{
//		__HAL_RCC_TIM2_CLK_ENABLE();            //ʹ��TIM3ʱ��
//		HAL_NVIC_SetPriority(TIM2_IRQn,1,3);    //�����ж����ȼ�����ռ���ȼ�1�������ȼ�3
//		HAL_NVIC_EnableIRQ(TIM2_IRQn);          //����ITM3�ж�   
//	}  
//}

//��ʱ��3�жϷ�����
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM2_Handler);
}

////��ʱ��3�жϷ���������,�ڶ�ʱ����
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if(htim==(&TIM2_Handler))
//    {
//     
//    }
//}

//����TIMͨ��4��ռ�ձ�
//compare:�Ƚ�ֵ
void TIM_SetTIM2Compare1(u32 compare)
{
//	 TIM3->CCER&=0x7fff;
//	__HAL_TIM_SET_COMPARE(&TIM2_Handler, TIM_CHANNEL_1,compare);
//	 TIM3->CCER&=0x1000;
	TIM2->CCR1=compare; 
//	TIM2_CH1Handler.Pulse=compare; 
//	HAL_TIM_PWM_ConfigChannel(&TIM2_Handler,&TIM2_CH1Handler,TIM_CHANNEL_1); 
	
}

void TIM_SetTIM2Compare2(u32 compare)
{

	TIM2->CCR2=compare; 

	
}
void TIM_SetTIM2Compare3(u32 compare)
{

	TIM2->CCR3=compare; 

	
}

void TIM_SetTIM2Compare4(u32 compare)
{

	TIM2->CCR4=compare; 

	
}
