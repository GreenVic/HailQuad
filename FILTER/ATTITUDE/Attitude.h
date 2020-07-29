#ifndef _ATTITUDE_H
#define _ATTITUDE_H
#include "math.h"
#include "MPU9250.h"
#include "global.h"
#include "hmc5983.h"

extern short aacx,aacy,aacz;	        		//加速度传感器原始数据
extern short gyrox,gyroy,gyroz;        		//陀螺仪原始数据 
extern s16 mx,my,mz;							//磁力计原始数据
extern short compOrigin[3];		        	//电子罗盘原始数据 
extern float BodyAngleRate[3],BodyAcc[3][3];		//从九轴传感器中获得的角速度（陀螺仪，弧度制）、加速度（加速度计）
extern float EarthAngleRate[3][3],EarthAcc[3][3];	//相对地面参考系的角速度，弧度制
extern float AngleRate[2][3],Angle[2][3];		//反馈补偿后的角加速度和角度，弧度制
extern float AngleHmr[2],AngleAcc[2][2];		//电子罗盘计算出的偏航角和加速度计计算出的横滚角、俯仰角，弧度制
extern int   Timer_Flag;

extern float GyroOrigin[3];     //陀螺仪输出的角加速度,弧度制
extern float AccOrigin[3];		//加速度计输出的加速度，单位为G（重力加速度）
extern float Mag[3];				//磁力计（转换后），单位为uT
extern float Mag_Max[3];			//磁力计最大值与最小值，用于校正
extern float Mag_Min[3];
extern float Angle_Accorigin[3][3];   //由加速度计得到的原始角度,弧度制
extern float Cali_Sum_Gyro[3];     //标定用的角速度和
extern float Cali_Sum_Acc[3];		//标定用的加速度和
extern float Cali_Sum_Angle[3];		 //标定用的加速度得出的角度和
extern float Average_Acc[3];			//高通滤波用的平均加速度
extern float Average_Gyro[3];			//平均加速度
extern float Zero_Angle[3];			//角度零点
extern int Cali_Length;     		//标定中的总次数
extern long Cali_Num;          //标定次数
extern float Mag_Center[3],Mag_Radius[3];	//磁力计各个方向的中心和半径（椭球拟合）即平移参数和缩放参数







double SafeSin(double tempnum);      //为了防止sin或cos里的参数超限,限制角度的绝对值小于等于pi/2
	
float SafeAsin(float tempnum);       //使得asin函数的变量处在(-1,1)，防止滤波出现发散
	
void ReadMPU9250(void);                  //从九轴传感器MPU9250中读取加速度、角加速度和磁感应强度
	
void SensorCalibration(void);			//传感器标定
void AttitudeFilter(void);				//姿态滤波











#endif
