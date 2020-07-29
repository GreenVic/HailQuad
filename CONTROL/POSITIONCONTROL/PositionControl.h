#ifndef _PositionControl_H
#define _PositionControl_H
#include "global.h"
#include "math.h"
#include "Velocity.h"
#include "Position.h"
#include "GroundControl.h"


extern int PosCtrFirst;
//��Ҫָʾ����
extern int GpsForPos;//ʹ��GPS����λ���˲�
//ָʾ�Ƿ������һ��
extern int CtrSN;
extern int CtrEW;
//����������
extern float SnGpsP;//GPSλ�ÿ��Ʊ���
extern float SnGpsD;
extern int SnGpsF;
extern float EwGpsP;
extern float EwGpsD;
extern int EwGpsF;

extern float PoControlSN[2];//λ�ÿ������
extern float PoControlEW[2];

extern float uP_vel_SN[3],uP_vel_EW[3]; 
extern float uP_pos_SN,uP_pos_EW; 

extern float AccSNFilter1;//����һ�׹����˲��Ľ��
extern float AccSNFilter2;//�������׹����˲��Ľ��
extern float AccEWFilter1;//����һ�׹����˲��Ľ��
extern float AccEWFilter2;//�������׹����˲��Ľ��

extern float uP_acc_SN;
extern float SnAccKp;
//extern float TrackTargetAccSN[2];
extern float uP_acc_EW;
extern float EwAccKp;
//extern float TrackTargetAccEW[2];

void PositionControl(void);
void SwitchPosCtrInit (void);

#endif
