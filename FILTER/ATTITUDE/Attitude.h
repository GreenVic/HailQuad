#ifndef _ATTITUDE_H
#define _ATTITUDE_H
#include "math.h"
#include "MPU9250.h"
#include "global.h"
#include "hmc5983.h"

extern short aacx,aacy,aacz;	        		//���ٶȴ�����ԭʼ����
extern short gyrox,gyroy,gyroz;        		//������ԭʼ���� 
extern s16 mx,my,mz;							//������ԭʼ����
extern short compOrigin[3];		        	//��������ԭʼ���� 
extern float BodyAngleRate[3],BodyAcc[3][3];		//�Ӿ��ᴫ�����л�õĽ��ٶȣ������ǣ������ƣ������ٶȣ����ٶȼƣ�
extern float EarthAngleRate[3][3],EarthAcc[3][3];	//��Ե���ο�ϵ�Ľ��ٶȣ�������
extern float AngleRate[2][3],Angle[2][3];		//����������ĽǼ��ٶȺͽǶȣ�������
extern float AngleHmr[2],AngleAcc[2][2];		//�������̼������ƫ���Ǻͼ��ٶȼƼ�����ĺ���ǡ������ǣ�������
extern int   Timer_Flag;

extern float GyroOrigin[3];     //����������ĽǼ��ٶ�,������
extern float AccOrigin[3];		//���ٶȼ�����ļ��ٶȣ���λΪG���������ٶȣ�
extern float Mag[3];				//�����ƣ�ת���󣩣���λΪuT
extern float Mag_Max[3];			//���������ֵ����Сֵ������У��
extern float Mag_Min[3];
extern float Angle_Accorigin[3][3];   //�ɼ��ٶȼƵõ���ԭʼ�Ƕ�,������
extern float Cali_Sum_Gyro[3];     //�궨�õĽ��ٶȺ�
extern float Cali_Sum_Acc[3];		//�궨�õļ��ٶȺ�
extern float Cali_Sum_Angle[3];		 //�궨�õļ��ٶȵó��ĽǶȺ�
extern float Average_Acc[3];			//��ͨ�˲��õ�ƽ�����ٶ�
extern float Average_Gyro[3];			//ƽ�����ٶ�
extern float Zero_Angle[3];			//�Ƕ����
extern int Cali_Length;     		//�궨�е��ܴ���
extern long Cali_Num;          //�궨����
extern float Mag_Center[3],Mag_Radius[3];	//�����Ƹ�����������ĺͰ뾶��������ϣ���ƽ�Ʋ��������Ų���







double SafeSin(double tempnum);      //Ϊ�˷�ֹsin��cos��Ĳ�������,���ƽǶȵľ���ֵС�ڵ���pi/2
	
float SafeAsin(float tempnum);       //ʹ��asin�����ı�������(-1,1)����ֹ�˲����ַ�ɢ
	
void ReadMPU9250(void);                  //�Ӿ��ᴫ����MPU9250�ж�ȡ���ٶȡ��Ǽ��ٶȺʹŸ�Ӧǿ��
	
void SensorCalibration(void);			//�������궨
void AttitudeFilter(void);				//��̬�˲�











#endif
