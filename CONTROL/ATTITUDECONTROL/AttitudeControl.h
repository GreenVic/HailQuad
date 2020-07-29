#ifndef _AttitudeControl_H
#define _AttitudeControl_H


#include "Attitude.h"
#include "math.h"


extern int AngCtrFirst;
extern float AttTrackFilterAng[3];
extern float AngleDelta[3][3];
extern float AngleRate_Delta[3][3];
extern float AngleRate_Gyro[3][3];
extern float ControlPitch;
extern float ControlRoll;
extern float ControlYaw;
extern float ControlCollective;
//���������
extern float ControlEW;
extern float ControlSN;
extern float ControlSnBody;
extern float ControlEwBody;
#define AttitudeBound 0.69813*0.75//�ڻ����ƽ�30��
//��̬�����
extern float PitchZero;//ֱ������ʵ�����
extern float RollZero;
extern float YawZero;
extern float PitchTargetZero;//��������������
extern float RollTargetZero;
extern float YawTargetZero;
//��ͨ�˲�������
extern float Filter_Gyro_F;
//����������
extern float PitchP;
extern float PitchI;
extern float PitchD;
extern int PitchF;
extern float RollP;
extern float RollI;
extern float RollD;
extern int RollF;
extern int YawP;
extern int YawI;
extern int YawD;
extern int YawF;
extern float yaw_integrate;
extern float yaw_error_bound;
extern float yaw_integrate_bound;

extern unsigned int ServoHL[6] ;//���ջ�����ֵ��������ջ�PWM�źŵĸߵ�ƽʱ�䣬1000~2000ms
//�������ķ�ΧΪ1000~2000ms����Ӧ�Ƕ�Ϊ-90~90�ȣ�����Ϊң����ָ��
extern unsigned int BServoHL;//����
extern unsigned int LServoHL;//����
extern unsigned int RServoHL;//�Ҷ��
extern unsigned int TServoHL;//β���
extern unsigned int PServoHL;//���Ŷ��



void AttitudeControl(void);
void ReceiveOrder(void);//����ң����ָ��
#endif


