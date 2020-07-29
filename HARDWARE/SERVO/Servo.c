#include "Servo.h"



TIM_HandleTypeDef TIM2_Handler;      	//定时器句柄 
TIM_OC_InitTypeDef TIM2_CH1Handler;     //定时器2通道1句柄
TIM_OC_InitTypeDef TIM2_CH2Handler;     //定时器2通道2句柄
TIM_OC_InitTypeDef TIM2_CH3Handler;     //定时器2通道3句柄
TIM_OC_InitTypeDef TIM2_CH4Handler;     //定时器2通道4句柄
//通用定时器3中断初始化,定时器3在APB1上，APB1的定时器时钟为200MHz
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!(定时器3挂在APB1上，时钟为HCLK/2)
void TIM2_Init(u16 arr,u16 psc)
{  
    TIM2_Handler.Instance=TIM2;                          //通用定时器3
    TIM2_Handler.Init.Prescaler=psc;                     //分频
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM2_Handler.Init.Period=arr;                        //自动装载值
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&TIM2_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM2_Handler); //使能定时器3和定时器3更新中断：TIM_IT_UPDATE    
}

//TIM3 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM2_PWM_Init(u32 arr,u16 psc)
{ 
	 TIM2_Handler.Instance=TIM2;            //定时器3
    TIM2_Handler.Init.Prescaler=psc;       //定时器分频
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
    TIM2_Handler.Init.Period=arr;          //自动重装载值
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM2_Handler);       //初始化PWM
    //通道1
    TIM2_CH1Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM2_CH1Handler.Pulse=10000;            //设置比较值,此值用来确定占空比，
                                            //默认比较值为自动重装载值的一半,即占空比为50%
    TIM2_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为高 
	TIM2_CH1Handler.OCFastMode=TIM_OCFAST_ENABLE;
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handler,&TIM2_CH1Handler,TIM_CHANNEL_1);//配置TIM2通道1
    HAL_TIM_PWM_Start(&TIM2_Handler,TIM_CHANNEL_1);//开启PWM通道1
	
	//通道2
	TIM2_CH2Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM2_CH2Handler.Pulse=10000;            //设置比较值,此值用来确定占空比，
                                            //默认比较值为自动重装载值的一半,即占空比为50%
    TIM2_CH2Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为高 
	TIM2_CH2Handler.OCFastMode=TIM_OCFAST_ENABLE;
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handler,&TIM2_CH2Handler,TIM_CHANNEL_2);//配置TIM2通道2
    HAL_TIM_PWM_Start(&TIM2_Handler,TIM_CHANNEL_2);//开启PWM通道2
	//通道3
	TIM2_CH3Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM2_CH3Handler.Pulse=10000;            //设置比较值,此值用来确定占空比，
                                            //默认比较值为自动重装载值的一半,即占空比为50%
    TIM2_CH3Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为高 
	TIM2_CH3Handler.OCFastMode=TIM_OCFAST_ENABLE;
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handler,&TIM2_CH3Handler,TIM_CHANNEL_3);//配置TIM2通道3
    HAL_TIM_PWM_Start(&TIM2_Handler,TIM_CHANNEL_3);//开启PWM通道3
	
	//通道4
	TIM2_CH4Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM2_CH4Handler.Pulse=10000;            //设置比较值,此值用来确定占空比，
                                            //默认比较值为自动重装载值的一半,即占空比为50%
    TIM2_CH4Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为高 
	TIM2_CH4Handler.OCFastMode=TIM_OCFAST_ENABLE;
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handler,&TIM2_CH4Handler,TIM_CHANNEL_4);//配置TIM2通道2
    HAL_TIM_PWM_Start(&TIM2_Handler,TIM_CHANNEL_4);//开启PWM通道4
}

//定时器底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_PWM_Init()调用
//htim:定时器句柄
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
   //通道1
    GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_TIM2_CLK_ENABLE();			//使能定时器2
    __HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOA时钟
	
    GPIO_Initure.Pin=GPIO_PIN_5;           	//PA5
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推完输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //高速
	GPIO_Initure.Alternate=GPIO_AF1_TIM2;	//PB1复用为TIM3_CH4
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	//通道2
	
    GPIO_Initure.Pin=GPIO_PIN_1;           	//PA1
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	//通道3
	
    GPIO_Initure.Pin=GPIO_PIN_2;           	//PA2
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	
	//通道4
	
    GPIO_Initure.Pin=GPIO_PIN_3;           	//PA3
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
}

//定时器底册驱动，开启时钟，设置中断优先级
//此函数会被HAL_TIM_Base_Init()函数调用
//void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
//{
//    if(htim->Instance==TIM2)
//	{
//		__HAL_RCC_TIM2_CLK_ENABLE();            //使能TIM3时钟
//		HAL_NVIC_SetPriority(TIM2_IRQn,1,3);    //设置中断优先级，抢占优先级1，子优先级3
//		HAL_NVIC_EnableIRQ(TIM2_IRQn);          //开启ITM3中断   
//	}  
//}

//定时器3中断服务函数
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM2_Handler);
}

////定时器3中断服务函数调用,在定时器中
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if(htim==(&TIM2_Handler))
//    {
//     
//    }
//}

//设置TIM通道4的占空比
//compare:比较值
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
