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
//控制器输出
extern float ControlEW;
extern float ControlSN;
extern float ControlSnBody;
extern float ControlEwBody;
#define AttitudeBound 0.69813*0.75//内环控制界30度
//姿态角零点
extern float PitchZero;//直升机的实际零点
extern float RollZero;
extern float YawZero;
extern float PitchTargetZero;//进入控制器的零点
extern float RollTargetZero;
extern float YawTargetZero;
//低通滤波器参数
extern float Filter_Gyro_F;
//控制器参数
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

extern unsigned int ServoHL[6] ;//接收机捕获值，代表接收机PWM信号的高电平时间，1000~2000ms
//舵机输入的范围为1000~2000ms，对应角度为-90~90度，以下为遥控器指令
extern unsigned int BServoHL;//后舵机
extern unsigned int LServoHL;//左舵机
extern unsigned int RServoHL;//右舵机
extern unsigned int TServoHL;//尾舵机
extern unsigned int PServoHL;//油门舵机



void AttitudeControl(void);
void ReceiveOrder(void);//接受遥控器指令
#endif


