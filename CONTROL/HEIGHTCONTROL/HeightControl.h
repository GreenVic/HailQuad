#ifndef HEIGHTCONTROL_H_
#define HEIGHTCONTROL_H_

extern float Height[3];//融合滤波后的高度,定义在Height.c中

int HighCtrFirst = 0;
int CtrHeight = 0;//指示是否控制高度
float TargetHeight[3];
float TargetHeightVel[2] = {0};
float TargetHeight_remote=0;
int SwitchHeight;
int CurrentHeight[3];
float HeightDelta[2];
float HeightDeltaVel[2];
float SumHeightDelta = 0.0;

//地面站发送的航迹值
int HeightTrack[2] = {0,0};
//汪雄峰
float HeightTrackStage = 0;//以前是int
float HeightTrackWhole = 0;//以前是int
//控制器参数
int HeightBound = 100;
int HeightP = 120;
int HeightD = 60;
int HeightF = 40;
int GpsHeightP = 100;
int GpsHeightD = 40;
int GpsHeightF = 20;
float ControlHeight[4] = {0,0,0,0};

int GpsHeightCtr = 1;//用GPS控制高度
float CurrentHeightOffset = 0;
extern int AutoriseFlag;//自主启动标志量,定义在TakeoffLand.c
extern float VelocityH[3];//垂直方向上的速度，定义在Height.c
int is_remote_move_heli = 0;
float Offset_VelTargetHeight[2]={0,0};
float RouteVelHeight[2] = {0,0};
float remote_move_height_stage = 0;
float remote_move_up_vel[4] = {0};
float Offset_TargetHeight[2]={0,0};

#endif
