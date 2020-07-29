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
	
	/*****************************************************初始化变量*****************************************************/
	u8 ManualPeriod = 0;  //手动状态周期指示和自动状态周期指示
    u8 SwitchFlag = 0; //自动状态和手动状态的切换变量。更换状态时需要更新一些变量的值，为了避免每个周期都进行更新，使用此切换变量    移至global.c
	

	u8 Time_Cnt;							//计时用
	
	Cache_Enable();                         //打开L1-Cache
	HAL_Init();				                //初始化HAL库
	Stm32_Clock_Init(160,5,2,4);            //设置时钟,400Mhz 
	delay_init(400);						//延时初始化
	uart_init(460800);						//初始化串口1
	MPU9250_Init();             			//初始化MPU9250	
	uart4_init(100000);						//初始化串口4
	TIM3_Init(250-1,2000-1);				//定时器3初始化，定时器时钟为200M，分频系数为2000-1，
										    //所以定时器3的频率为200M/2000=100K，自动重装载为250-1，那么定时器周期就是2.5ms
	TIM2_PWM_Init(25000-1,20-1);           //舵机用的PWM,    200M分频系数为20，频率为10MHZ
	USART3_init(115200);					//串口3初始化
	MYDMA_Config();							//初始化DMA
	hmc5983_init();                          //初始化hmc5983
	
	
	
	 
	
	SensorCalibration();
//	Gps_Cali();                             //检测是否有gps且数据是否合格，如果没有Gps则会在里面循环，所以测试时需要注释掉！
	Remoter_Cali();
//	printf("%d,%d,%d,%d;\r\n",Cali_Channel[0],Cali_Channel[1],Cali_Channel[2],Cali_Channel[3]);	
	while(1)
	{	
		
		/////////////////////////读取SBUS数据////////////////////////////////////////////		
		if((__HAL_UART_GET_FLAG(&UART4_Handler,UART_FLAG_IDLE) != RESET))        //判断串口空闲    PA11
		{
			Sbus_Recieve();                        //DMA接收遥控器SBUS数据,根据空闲中断判断。最终数据存入Remoter_Channel[16]
			
		}
	
		
		/////////////////////////读取GPS数据////////////////////////////////////////////
		if((__HAL_UART_GET_FLAG(&USART3_Handler,UART_FLAG_IDLE) != RESET))  //idle标志被置位     PB11
		{
			Gps_Recieve();               		   //DMA接收，处理GPS数据。最终数据存入gps结构体
		}
	

		
		/////////////////////////姿态滤波与控制//////////////////////////////////////////
		//主循环400Hz
		if(Timer_Flag)
		{
			Time_Cnt++;
			AttitudeFilter();
			Timer_Flag = 0;
//			AttitudeControl();
//			heightFilter();	

			/*****************************************************测试区域*****************************************************/
//			printf("%10f,%10f,%10f;\r\n",Angle[0][0]/DtoR,Angle[0][1]/DtoR,Angle[0][2]/DtoR);
//			printf("%f;\r\n",AngleHmr[0]/DtoR);
//			printf("%10f,%10f,%10f,%10f;\r\n",AngleHmr[0]/DtoR,Angle[0][0]/DtoR,Angle[0][1]/DtoR,GyroOrigin[2]/DtoR);
//			printf("%f,%f,%f;\r\n",4800*mx/65536.0,4800*my/65536.0,4800*mz/65536.0);     //罗盘校准用
//			printf("%f,%f,%f,%f;\r\n",BodyAcc[0][2],EarthAcc[0][2],Angle[0][0]/DtoR,Angle[0][1]/DtoR);//z轴加速度
//			printf("%10f,%10f,%10f,%10f;\r\n",AngleRate[0][0]/DtoR ,AngleRate[0][1]/DtoR,EarthAngleRate[0][2]/DtoR,AngleHmr[0]/DtoR);
//		    printf("%d,%d,%d,%d;\r\n",LServoOut,RServoOut,BServoOut,TServoOut);
//			printf("%f,%f,%f;\r\n",ControlPitch,ControlRoll,ControlYaw);
//			printf("%f,%f,%f;\r\n",BodyAcc[0][0],BodyAcc[0][1],BodyAcc[0][2]);	
//			VelocityFilter(gps1);
			
			
			/*****************************************************手动状态*****************************************************/           
            if(Remoter_Channel[8] < 16500 || AutoStateFlag == 0)   ///遥控器指令为手动状态
            //if(ServoHL[4] < 155000 || AutoStateFlag == 0)//260
            {   
                if(ManualPeriod < 3)ManualPeriod ++; 
                else//确实切入手动而非捕获干扰
                {
                    if(SwitchFlag == 0)//自动状态和手动状态的切换变量。更换状态时需要更新一些变量的值，为了避免每个周期都进行更新，使用此切换变量。以做到此段代码只执行一次                     
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

		
		if(Time_Cnt>49)   //不同周期的循环，暂无用
		{Time_Cnt=0;}
		
		}
		
	}
}


