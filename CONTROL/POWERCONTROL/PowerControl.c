#include "PowerControl.h"
#include "Servo.h"
#include "SBUS.h"


//飞控给出的舵机输入
unsigned int MOTOR_M1;//右上↗
unsigned int MOTOR_M2;//左下↙
unsigned int MOTOR_M3;//左上↖
unsigned int MOTOR_M4;//右下↘
float ControlHeight1;

void ReceiveOrder(void)//接受遥控器指令
{
//	BServoHL = ServoHL[1];
//    LServoHL = ServoHL[5];
//    RServoHL = ServoHL[0];
//    TServoHL = ServoHL[3];
//    PServoHL = ServoHL[2];
	BServoHL = 1500;
    LServoHL = 2000;
    RServoHL = 1000;
	ControlEW = (-0.5774 * RServoHL + 0.5774 * LServoHL - 1500) / 500 * 90 * DtoR;
	ControlSN = (-0.3333 * RServoHL + 0.6667 * BServoHL -0.3333 * LServoHL - 1500) / 500 * 90 * DtoR;
	ControlCollective = (0.3333 * RServoHL + 0.3333 * BServoHL + 0.3333 * LServoHL - 1500) / 500 * 90 * DtoR;
}
void Motor_Set(void)      //电机分配
{
	ControlHeight1 = (Remoter_Channel[2] - Cali_Channel[2])/119.0f;   //除以一个数使得油门量程对应1-2ms
	MOTOR_M1 = 10000+ (ControlYaw - Ratio*ControlRoll - Ratio*ControlPitch + ControlHeight1 ) / DtoR / 90 *1500;
	MOTOR_M2 = 10000+ (ControlYaw + Ratio*ControlRoll + Ratio*ControlPitch + ControlHeight1) / DtoR / 90 *1500;
	MOTOR_M3 = 10000+ (ControlYaw *(-1) + Ratio*ControlRoll - Ratio*ControlPitch + ControlHeight1) / DtoR / 90 *1500;
	MOTOR_M4 = 10000+ (ControlYaw *(-1) - Ratio*ControlRoll + Ratio*ControlPitch + ControlHeight1) / DtoR / 90 *1500;
	
	
	//电机限幅
	if(MOTOR_M1 > 20000)MOTOR_M1 = 20000;
    if(MOTOR_M1 < 10000)MOTOR_M1 = 10000;
    if(MOTOR_M2 > 20000)MOTOR_M2 = 20000;
    if(MOTOR_M2 < 10000)MOTOR_M2 = 10000;
    if(MOTOR_M3 > 20000)MOTOR_M3 = 20000;
    if(MOTOR_M3 < 10000)MOTOR_M3 = 10000;
	if(MOTOR_M4 > 20000)MOTOR_M4 = 20000;
    if(MOTOR_M4 < 10000)MOTOR_M4 = 10000;


}

void Motor_PWMSet( unsigned int m1_set,unsigned int m2_set, unsigned int m3_set, unsigned int m4_set)    //PWM电机调速
{

	TIM_SetTIM2Compare1(m1_set);                  
	TIM_SetTIM2Compare2(m2_set);			 
	TIM_SetTIM2Compare3(m3_set);				
	TIM_SetTIM2Compare4(m4_set);				 




}
	










