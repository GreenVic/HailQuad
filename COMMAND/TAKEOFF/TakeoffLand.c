#include "TakeoffLand.h"




int IsFirstEnterTakeOff = 1;
extern unsigned int ServoHL[6] ;//���ջ�����ֵ��������ջ�PWM�źŵĸߵ�ƽʱ�䣬1000~2000ms,������AttitudeControl.c��
float CollectiveStep1 = 0;
float CollectiveStep2 = 0;
float CollectiveTmp = 0;
int AutoriseFlag;//����������־��
int AutoriseFlag2;//����������־��
unsigned int BServoSw;
unsigned int LServoSw;
unsigned int RServoSw;//������HeightControl

float TakeOffHeight;
float TakeOffVel;
float LandHeight;
float LandVel;
int IsDescend;

int IsFirstEnterGenerateHeightVel = 1;
int IsAccelerateRise = 1;
//float HeightTrackStage = 0;//��ǰ��int

void AutoTakeOff(void)
{
	static float CollectiveSwitch = 0;
    static int cnt = 0;
    //printf("start take off\n");
    if(IsFirstEnterTakeOff)
    {
        IsFirstEnterTakeOff = 0;
        CollectiveSwitch =( (ServoHL[1] - BServoStatic2)* B_direction + (ServoHL[5] - LServoStatic2)* L_direction +\
                       R_direction*(ServoHL[0] - RServoStatic2))/3;  //���㵱ǰ���ܾ�
        CollectiveStep1 = (CollectiveStage1 - CollectiveSwitch)/50.0/TimeStage1; //�����һ�׶�ÿ�����ڵ�����
        CollectiveStep2 = (CollectiveStage2 - CollectiveStage1)/50.0/TimeStage2; //����ڶ��׶�ÿ�����ڵ�����
        CollectiveTmp = 0;//������л��ܾ�ĵ�ǰ�ܾ�����
        cnt = 0;
    }
    cnt ++;
    //printf("%f %f %f\n",CollectiveSwitch,CollectiveStage1,CollectiveTmp);
    if(cnt < TimeStage1 * 50)//��һ�׶�
    {
        CollectiveTmp += CollectiveStep1;
    }
    else if(cnt < (TimeStage1 + TimeStage2) * 50 )//�ڶ��׶�
    {
        CollectiveTmp += CollectiveStep2;
    }
    else 
    {
        AutoriseFlag = 0;//��ʾ�����Ľ���
        AutoriseFlag2 = 1;//����ջ����ٶȴӵ�ǰֵ��ʼ�ӣ��߶�����
        BServoSw = BServoHL ;//������Ҫ����ջ���׼������л�ֵ
        LServoSw = LServoHL ;
        RServoSw = RServoHL ;
        //PServoSwitch = PServoHL ;//��˧���
        return;
    }
    //printf("%f\n",CollectiveTmp);
    BServoHL = BServoSw + B_direction * CollectiveTmp ;//������PWM
    LServoHL = LServoSw + L_direction * CollectiveTmp ;
    RServoHL = RServoSw + R_direction * CollectiveTmp ; 
    //PServoHL=  PServoSwitch + P_direction * CollectiveTmp*3; //��˧���
    //printf("%f %f %f",CollectiveTmp,0,0);
}

void AutoLand()
{
}


void GenerateTakeOffHeightVel()
{
//	if(IsFirstEnterGenerateHeightVel )
//    {
//        IsFirstEnterGenerateHeightVel = 0;
//        TakeOffVel = 0;//ѧ������ΪUltrasoundNewHeightVel[0];�ο��ٶȴӵ�ǰ�ٶȿ�ʼ����
//    }
//	if(IsAccelerateRise)//���������׶�
//    {
//        if(TakeOffVel < MaxHeightVel)//�ٶ�С������ٶȣ��ٶ���������
//        {
//            TakeOffVel += TakeOffAcc * SystemPeriod;
//        }
//        else
//        {
//            IsAccelerateRise = 0;//���ٵ�����ٶȺ�״̬��Ϊ��������
//        }
//    }
//	 else//���������׶�
//    {
//        if(TakeOffVel > 0)//�ٶȴ�����ʱ���ٶ�������С 
//        {
//            TakeOffVel -= TakeOffAcc * SystemPeriod;
//        }
//        else
//        {
//            TakeOffVel = 0;
//            AutoriseFlag2 = 0;//�ٶȼ�����󣬽� �����ĵĸ߶ȼӵ� stage
//            HeightTrackStage += TakeOffHeight * 100;
//            
//            //��������ɺ���ΪGPS����
//            OperateSelect = OperateSelect | 0x0400;//GPS��־��1
//            OperateSelect2 = OperateSelect2 & 0xfffb;//MsAccHeightCtr��־��0
//            UltrasoundCtr = 0;
//            GpsHeightCtr = 1;
//            MsAccHeightCtr = 0;
//            CurrentHeightOffset = CurrentHeight[1] - GpsAccHeight[0]*1000;
//            //��������ɺ�ʼִ�к���
//            if(TotalTraceNum > 0 && InRoute == 0)//���֮ǰ�Ѿ��յ��˺�������������δִ�������ڲ���ִ�к���
//            {
//                DestStaticPoint();//�������ǰ��������������ָ����γ�ȵ㣩�Ĳ���
//                InRoute = 1;//���������У��ܾ������κ�·������
//                RouteStop = 0;
//                CurrentTraceNum = 0; //���õ�ǰ��ִ�еĹ켣
//                CurrentRouteFinish = 1;//׼����ʼ��1���켣�ĳ�ʼ��
//            }
//        }
//    }
//    TakeOffHeight += TakeOffVel * SystemPeriod;
}
