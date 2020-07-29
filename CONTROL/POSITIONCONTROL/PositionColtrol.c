#include "PositionControl.h"




int PosCtrFirst=0;
//��Ҫָʾ����
int GpsForPos = 0;//ʹ��GPS����λ���˲�
//ָʾ�Ƿ������һ��
int CtrSN = 0 ;
int CtrEW = 0;
//����������
float SnGpsP = 4 * DtoR;//GPSλ�ÿ��Ʊ���
float SnGpsD = 30 * DtoR;
int SnGpsF = 40;
float EwGpsP = 4  * DtoR;
float EwGpsD = 30 * DtoR;
int EwGpsF = 40;

float PosControlSN[2] = {0,0};//λ�ÿ������
float PosControlEW[2] = {0,0};

float uP_pos_SN = 0,uP_pos_EW = 0; 
float uP_vel_SN[3] = {0,0,0},uP_vel_EW[3] = {0,0,0}; 

float uP_acc_SN = 0;
float SnAccKp = 0;
//float TrackTargetAccSN[2] = {0,0};
float uP_acc_EW = 0;
float EwAccKp = 0;
//float TrackTargetAccEW[2] = {0,0};


void PositionControl(void)//λ�ÿ����Ӻ���
{
    if(PosCtrFirst == 0)
    {
        PosCtrFirst = 1;
        SwitchPosCtrInit();
   
    }  
    if(GpsForPos == 1)
    {                   
        //SN����
        uP_acc_SN = SnAccKp * (TargetAccSN - Filter_AccSN2);
        uP_acc_EW = EwAccKp * (TargetAccEW - Filter_AccEW2);
        /*λ�û�P����*/  
        uP_pos_SN = (PosTargetSN[0]+Offset_PosTargetSN - BodyPosSN[0]) * SnGpsP;//BodyPosSN[0]
        /*�ٶȻ�P����*/
        uP_vel_SN[2] =uP_vel_SN[1];
        uP_vel_SN[1] =uP_vel_SN[0];
        uP_vel_SN[0] = ((2 - SnGpsF * SystemPeriod) * uP_vel_SN[1] + SnGpsD * SnGpsF * SystemPeriod * (VelTargetSN[0]+Offset_VelTargetSN[0] + VelTargetSN[1]+Offset_VelTargetSN[1] - VelocitySN[0] - VelocitySN[1]))/(2 + SnGpsF * SystemPeriod);
        		
		PosControlSN[1] = PosControlSN[0];
        PosControlSN[0] = uP_pos_SN + uP_vel_SN[0] + uP_acc_SN;
		
		//EW����
        /*λ�û�P����*/  
        uP_pos_EW = (PosTargetEW[0]+Offset_PosTargetEW - BodyPosEW[0]) * EwGpsP;//BodyPosEW[0]
        /*�ٶȻ�P����*/
        uP_vel_EW[2] =uP_vel_EW[1];
        uP_vel_EW[1] =uP_vel_EW[0];
        uP_vel_EW[0] = ((2 - EwGpsF * SystemPeriod) * uP_vel_EW[0] + EwGpsD * EwGpsF * SystemPeriod * (VelTargetEW[0]+Offset_VelTargetEW[0] + VelTargetEW[1]+Offset_VelTargetEW[1] - VelocitySN[0] - VelocitySN[1]))/(2 + EwGpsF * SystemPeriod);       

		PosControlEW[1] = PosControlEW[0];
        PosControlEW[0] = uP_pos_EW + uP_vel_EW[0] + uP_acc_EW;
	
	}
}
void SwitchPosCtrInit(void)
{ 
    
    /*����������ֵ*/
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
