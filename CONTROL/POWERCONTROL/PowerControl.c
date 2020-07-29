#include "PowerControl.h"
#include "Servo.h"
#include "SBUS.h"


//�ɿظ����Ķ������
unsigned int MOTOR_M1;//���ϨJ
unsigned int MOTOR_M2;//���¨L
unsigned int MOTOR_M3;//���ϨI
unsigned int MOTOR_M4;//���¨K
float ControlHeight1;

void ReceiveOrder(void)//����ң����ָ��
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
void Motor_Set(void)      //�������
{
	ControlHeight1 = (Remoter_Channel[2] - Cali_Channel[2])/119.0f;   //����һ����ʹ���������̶�Ӧ1-2ms
	MOTOR_M1 = 10000+ (ControlYaw - Ratio*ControlRoll - Ratio*ControlPitch + ControlHeight1 ) / DtoR / 90 *1500;
	MOTOR_M2 = 10000+ (ControlYaw + Ratio*ControlRoll + Ratio*ControlPitch + ControlHeight1) / DtoR / 90 *1500;
	MOTOR_M3 = 10000+ (ControlYaw *(-1) + Ratio*ControlRoll - Ratio*ControlPitch + ControlHeight1) / DtoR / 90 *1500;
	MOTOR_M4 = 10000+ (ControlYaw *(-1) - Ratio*ControlRoll + Ratio*ControlPitch + ControlHeight1) / DtoR / 90 *1500;
	
	
	//����޷�
	if(MOTOR_M1 > 20000)MOTOR_M1 = 20000;
    if(MOTOR_M1 < 10000)MOTOR_M1 = 10000;
    if(MOTOR_M2 > 20000)MOTOR_M2 = 20000;
    if(MOTOR_M2 < 10000)MOTOR_M2 = 10000;
    if(MOTOR_M3 > 20000)MOTOR_M3 = 20000;
    if(MOTOR_M3 < 10000)MOTOR_M3 = 10000;
	if(MOTOR_M4 > 20000)MOTOR_M4 = 20000;
    if(MOTOR_M4 < 10000)MOTOR_M4 = 10000;


}

void Motor_PWMSet( unsigned int m1_set,unsigned int m2_set, unsigned int m3_set, unsigned int m4_set)    //PWM�������
{

	TIM_SetTIM2Compare1(m1_set);                  
	TIM_SetTIM2Compare2(m2_set);			 
	TIM_SetTIM2Compare3(m3_set);				
	TIM_SetTIM2Compare4(m4_set);				 




}
	










