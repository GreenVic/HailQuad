#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "MPU9250.h"
#include "IIC.h"
#include "math.h"
#include "Attitude.h"
#include "timer.h"
#include "AttitudeControl.h"
#include "Servo.h"
#include "gps.h"
#include "dma.h"
#include "Velocity.h"
#include "Height.h"
#include "string.h"
#include "SBUS.h"
#include "PowerControl.h"

int Timer_Flag;


//0-Roll 1-Pitch 2-Yaw
int main(void)
{
	
	/*****************************************************��ʼ������*****************************************************/
	u8 ManualPeriod = 0;  //�ֶ�״̬����ָʾ���Զ�״̬����ָʾ
    u8 SwitchFlag = 0; //�Զ�״̬���ֶ�״̬���л�����������״̬ʱ��Ҫ����һЩ������ֵ��Ϊ�˱���ÿ�����ڶ����и��£�ʹ�ô��л�����    ����global.c
	

	u8 Time_Cnt;							//��ʱ��
	
	Cache_Enable();                         //��L1-Cache
	HAL_Init();				                //��ʼ��HAL��
	Stm32_Clock_Init(160,5,2,4);            //����ʱ��,400Mhz 
	delay_init(400);						//��ʱ��ʼ��
	uart_init(460800);						//��ʼ������1
	MPU9250_Init();             			//��ʼ��MPU9250	
	uart4_init(100000);						//��ʼ������4
	TIM3_Init(250-1,2000-1);				//��ʱ��3��ʼ������ʱ��ʱ��Ϊ200M����Ƶϵ��Ϊ2000-1��
										    //���Զ�ʱ��3��Ƶ��Ϊ200M/2000=100K���Զ���װ��Ϊ250-1����ô��ʱ�����ھ���2.5ms
	TIM2_PWM_Init(25000-1,20-1);           //����õ�PWM,    200M��Ƶϵ��Ϊ20��Ƶ��Ϊ10MHZ
	USART3_init(115200);					//����3��ʼ��
	MYDMA_Config();							//��ʼ��DMA
	hmc5983_init();                          //��ʼ��hmc5983
	
	
	
	 
	
	SensorCalibration();
//	Gps_Cali();                             //����Ƿ���gps�������Ƿ�ϸ����û��Gps���������ѭ�������Բ���ʱ��Ҫע�͵���
	Remoter_Cali();
//	printf("%d,%d,%d,%d;\r\n",Cali_Channel[0],Cali_Channel[1],Cali_Channel[2],Cali_Channel[3]);	
	while(1)
	{	
		
		/////////////////////////��ȡSBUS����////////////////////////////////////////////		
		if((__HAL_UART_GET_FLAG(&UART4_Handler,UART_FLAG_IDLE) != RESET))        //�жϴ��ڿ���    PA11
		{
			Sbus_Recieve();                        //DMA����ң����SBUS����,���ݿ����ж��жϡ��������ݴ���Remoter_Channel[16]
			
		}
	
		
		/////////////////////////��ȡGPS����////////////////////////////////////////////
		if((__HAL_UART_GET_FLAG(&USART3_Handler,UART_FLAG_IDLE) != RESET))  //idle��־����λ     PB11
		{
			Gps_Recieve();               		   //DMA���գ�����GPS���ݡ��������ݴ���gps�ṹ��
		}
	

		
		/////////////////////////��̬�˲������//////////////////////////////////////////
		//��ѭ��400Hz
		if(Timer_Flag)
		{
			Time_Cnt++;
			AttitudeFilter();
			Timer_Flag = 0;
//			AttitudeControl();
//			heightFilter();	

			/*****************************************************��������*****************************************************/
//			printf("%10f,%10f,%10f;\r\n",Angle[0][0]/DtoR,Angle[0][1]/DtoR,Angle[0][2]/DtoR);
//			printf("%f;\r\n",AngleHmr[0]/DtoR);
//			printf("%10f,%10f,%10f,%10f;\r\n",AngleHmr[0]/DtoR,Angle[0][0]/DtoR,Angle[0][1]/DtoR,GyroOrigin[2]/DtoR);
//			printf("%f,%f,%f;\r\n",4800*mx/65536.0,4800*my/65536.0,4800*mz/65536.0);     //����У׼��
//			printf("%f,%f,%f,%f;\r\n",BodyAcc[0][2],EarthAcc[0][2],Angle[0][0]/DtoR,Angle[0][1]/DtoR);//z����ٶ�
//			printf("%10f,%10f,%10f,%10f;\r\n",AngleRate[0][0]/DtoR ,AngleRate[0][1]/DtoR,EarthAngleRate[0][2]/DtoR,AngleHmr[0]/DtoR);
//		    printf("%d,%d,%d,%d;\r\n",LServoOut,RServoOut,BServoOut,TServoOut);
//			printf("%f,%f,%f;\r\n",ControlPitch,ControlRoll,ControlYaw);
//			printf("%f,%f,%f;\r\n",BodyAcc[0][0],BodyAcc[0][1],BodyAcc[0][2]);	
//			VelocityFilter(gps1);
			
			
			/*****************************************************�ֶ�״̬*****************************************************/           
            if(Remoter_Channel[8] < 16500 || AutoStateFlag == 0)   ///ң����ָ��Ϊ�ֶ�״̬
            //if(ServoHL[4] < 155000 || AutoStateFlag == 0)//260
            {   
                if(ManualPeriod < 3)ManualPeriod ++; 
                else//ȷʵ�����ֶ����ǲ������
                {
                    if(SwitchFlag == 0)//�Զ�״̬���ֶ�״̬���л�����������״̬ʱ��Ҫ����һЩ������ֵ��Ϊ�˱���ÿ�����ڶ����и��£�ʹ�ô��л��������������˶δ���ִֻ��һ��                     
                    {
                        SwitchFlag = 1;
//                      AutoTakeOffLandInit();
                        if(Remoter_Channel[8] < 16500 )AutoStateFlag = 1;
                    }                 
					AttitudeControl();
					Motor_Set();
					Motor_PWMSet( MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4);
					printf("%d,%d,%d,%d;\r\n",MOTOR_M1,MOTOR_M2,MOTOR_M3,MOTOR_M4);	
			
		        }
			}

		
		if(Time_Cnt>49)   //��ͬ���ڵ�ѭ����������
		{Time_Cnt=0;}
		
		}
		
	}
}


