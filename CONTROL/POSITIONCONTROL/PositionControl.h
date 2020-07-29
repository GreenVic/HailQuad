#ifndef _PositionControl_H
#define _PositionControl_H
#include "global.h"
#include "math.h"
#include "Velocity.h"
#include "Position.h"
#include "GroundControl.h"


extern int PosCtrFirst;
//重要指示变量
extern int GpsForPos;//使用GPS进行位置滤波
//指示是否控制哪一轴
extern int CtrSN;
extern int CtrEW;
//控制器参数
extern float SnGpsP;//GPS位置控制变量
extern float SnGpsD;
extern int SnGpsF;
extern float EwGpsP;
extern float EwGpsD;
extern int EwGpsF;

extern float PoControlSN[2];//位置控制输出
extern float PoControlEW[2];

extern float uP_vel_SN[3],uP_vel_EW[3]; 
extern float uP_pos_SN,uP_pos_EW; 

extern float AccSNFilter1;//经过一阶惯性滤波的结果
extern float AccSNFilter2;//经过二阶惯性滤波的结果
extern float AccEWFilter1;//经过一阶惯性滤波的结果
extern float AccEWFilter2;//经过二阶惯性滤波的结果

extern float uP_acc_SN;
extern float SnAccKp;
//extern float TrackTargetAccSN[2];
extern float uP_acc_EW;
extern float EwAccKp;
//extern float TrackTargetAccEW[2];

void PositionControl(void);
void SwitchPosCtrInit (void);

#endif
