#ifndef HEIGHTCONTROL_H_
#define HEIGHTCONTROL_H_

extern float Height[3];//�ں��˲���ĸ߶�,������Height.c��

int HighCtrFirst = 0;
int CtrHeight = 0;//ָʾ�Ƿ���Ƹ߶�
float TargetHeight[3];
float TargetHeightVel[2] = {0};
float TargetHeight_remote=0;
int SwitchHeight;
int CurrentHeight[3];
float HeightDelta[2];
float HeightDeltaVel[2];
float SumHeightDelta = 0.0;

//����վ���͵ĺ���ֵ
int HeightTrack[2] = {0,0};
//���۷�
float HeightTrackStage = 0;//��ǰ��int
float HeightTrackWhole = 0;//��ǰ��int
//����������
int HeightBound = 100;
int HeightP = 120;
int HeightD = 60;
int HeightF = 40;
int GpsHeightP = 100;
int GpsHeightD = 40;
int GpsHeightF = 20;
float ControlHeight[4] = {0,0,0,0};

int GpsHeightCtr = 1;//��GPS���Ƹ߶�
float CurrentHeightOffset = 0;
extern int AutoriseFlag;//����������־��,������TakeoffLand.c
extern float VelocityH[3];//��ֱ�����ϵ��ٶȣ�������Height.c
int is_remote_move_heli = 0;
float Offset_VelTargetHeight[2]={0,0};
float RouteVelHeight[2] = {0,0};
float remote_move_height_stage = 0;
float remote_move_up_vel[4] = {0};
float Offset_TargetHeight[2]={0,0};

#endif
