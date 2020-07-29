#include "PositionControl.h"




int PosCtrFirst=0;
//重要指示变量
int GpsForPos = 0;//使用GPS进行位置滤波
//指示是否控制哪一轴
int CtrSN = 0 ;
int CtrEW = 0;
//控制器参数
float SnGpsP = 4 * DtoR;//GPS位置控制变量
float SnGpsD = 30 * DtoR;
int SnGpsF = 40;
float EwGpsP = 4  * DtoR;
float EwGpsD = 30 * DtoR;
int EwGpsF = 40;

float PosControlSN[2] = {0,0};//位置控制输出
float PosControlEW[2] = {0,0};

float uP_pos_SN = 0,uP_pos_EW = 0; 
float uP_vel_SN[3] = {0,0,0},uP_vel_EW[3] = {0,0,0}; 

float uP_acc_SN = 0;
float SnAccKp = 0;
//float TrackTargetAccSN[2] = {0,0};
float uP_acc_EW = 0;
float EwAccKp = 0;
//float TrackTargetAccEW[2] = {0,0};


void PositionControl(void)//位置控制子函数
{
    if(PosCtrFirst == 0)
    {
        PosCtrFirst = 1;
        SwitchPosCtrInit();
   
    }  
    if(GpsForPos == 1)
    {                   
        //SN轴向
        uP_acc_SN = SnAccKp * (TargetAccSN - Filter_AccSN2);
        uP_acc_EW = EwAccKp * (TargetAccEW - Filter_AccEW2);
        /*位置环P控制*/  
        uP_pos_SN = (PosTargetSN[0]+Offset_PosTargetSN - BodyPosSN[0]) * SnGpsP;//BodyPosSN[0]
        /*速度环P控制*/
        uP_vel_SN[2] =uP_vel_SN[1];
        uP_vel_SN[1] =uP_vel_SN[0];
        uP_vel_SN[0] = ((2 - SnGpsF * SystemPeriod) * uP_vel_SN[1] + SnGpsD * SnGpsF * SystemPeriod * (VelTargetSN[0]+Offset_VelTargetSN[0] + VelTargetSN[1]+Offset_VelTargetSN[1] - VelocitySN[0] - VelocitySN[1]))/(2 + SnGpsF * SystemPeriod);
        		
		PosControlSN[1] = PosControlSN[0];
        PosControlSN[0] = uP_pos_SN + uP_vel_SN[0] + uP_acc_SN;
		
		//EW轴向
        /*位置环P控制*/  
        uP_pos_EW = (PosTargetEW[0]+Offset_PosTargetEW - BodyPosEW[0]) * EwGpsP;//BodyPosEW[0]
        /*速度环P控制*/
        uP_vel_EW[2] =uP_vel_EW[1];
        uP_vel_EW[1] =uP_vel_EW[0];
        uP_vel_EW[0] = ((2 - EwGpsF * SystemPeriod) * uP_vel_EW[0] + EwGpsD * EwGpsF * SystemPeriod * (VelTargetEW[0]+Offset_VelTargetEW[0] + VelTargetEW[1]+Offset_VelTargetEW[1] - VelocitySN[0] - VelocitySN[1]))/(2 + EwGpsF * SystemPeriod);       

		PosControlEW[1] = PosControlEW[0];
        PosControlEW[0] = uP_pos_EW + uP_vel_EW[0] + uP_acc_EW;
	
	}
}
void SwitchPosCtrInit(void)
{ 
    
    /*控制量赋初值*/
    uP_vel_EW[0] = 0.0;
    uP_vel_EW[1] = 0.0;
    uP_vel_EW[2] = 0.0;        
    PosControlEW[0] = 0.0;
    PosControlEW[1] = 0.0; 
    
    uP_vel_SN[0] = 0.0;
    uP_vel_SN[1] = 0.0;
    uP_vel_SN[2] = 0.0;        
    PosControlSN[0] = 0.0; 
    PosControlSN[1] = 0.0;
    
}
