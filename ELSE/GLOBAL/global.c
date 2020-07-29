#include "global.h"
#include "sys.h"  


//short aacx,aacy,aacz;	        		
//short gyrox,gyroy,gyroz;        		
//short mx,my,mz;							
//short compOrigin[3],compOrigin1[3];		        
//float BodyAngleRate[3],BodyAcc[3];		
//float EarthAngleRate[3][3],EarthAcc[3];
//float AngleRate[2][3],Angle[2][3];		
//float AngleHmr[2],AngleAcc[2][2];		
//int Timer_Flag;					

//float GyroOrigin[3];    
//float AccOrigin[3];		
//float Mag[3];				
//float Mag_Max[3];			
//float Mag_Min[3];
//float Angle_Accorigin[3][3];  
//float Cali_Sum_Gyro[3];    
//float Cali_Sum_Angle[3];		 
//float Average_Gyro[3];			
//float Zero_Angle[3];			
//int Cali_Length = 100;     		
//long Cali_Num = 0; 

u8 AutoStateFlag = 1;// 手动时为1，切自动要满足两个条件 （Servo[4]>...  AutoStateFlag==1）,GPS失效时将该值改为0，退出自动
