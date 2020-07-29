#include "TakeoffLand.h"




int IsFirstEnterTakeOff = 1;
extern unsigned int ServoHL[6] ;//接收机捕获值，代表接收机PWM信号的高电平时间，1000~2000ms,定义在AttitudeControl.c中
float CollectiveStep1 = 0;
float CollectiveStep2 = 0;
float CollectiveTmp = 0;
int AutoriseFlag;//自主启动标志量
int AutoriseFlag2;//自主启动标志量
unsigned int BServoSw;
unsigned int LServoSw;
unsigned int RServoSw;//定义在HeightControl

float TakeOffHeight;
float TakeOffVel;
float LandHeight;
float LandVel;
int IsDescend;

int IsFirstEnterGenerateHeightVel = 1;
int IsAccelerateRise = 1;
//float HeightTrackStage = 0;//以前是int

void AutoTakeOff(void)
{
	static float CollectiveSwitch = 0;
    static int cnt = 0;
    //printf("start take off\n");
    if(IsFirstEnterTakeOff)
    {
        IsFirstEnterTakeOff = 0;
        CollectiveSwitch =( (ServoHL[1] - BServoStatic2)* B_direction + (ServoHL[5] - LServoStatic2)* L_direction +\
                       R_direction*(ServoHL[0] - RServoStatic2))/3;  //计算当前的总距
        CollectiveStep1 = (CollectiveStage1 - CollectiveSwitch)/50.0/TimeStage1; //计算第一阶段每个周期的增量
        CollectiveStep2 = (CollectiveStage2 - CollectiveStage1)/50.0/TimeStage2; //计算第二阶段每个周期的增量
        CollectiveTmp = 0;//相对于切换总距的当前总距增量
        cnt = 0;
    }
    cnt ++;
    //printf("%f %f %f\n",CollectiveSwitch,CollectiveStage1,CollectiveTmp);
    if(cnt < TimeStage1 * 50)//第一阶段
    {
        CollectiveTmp += CollectiveStep1;
    }
    else if(cnt < (TimeStage1 + TimeStage2) * 50 )//第二阶段
    {
        CollectiveTmp += CollectiveStep2;
    }
    else 
    {
        AutoriseFlag = 0;//表示开环的结束
        AutoriseFlag2 = 1;//进入闭环，速度从当前值开始加，高度上升
        BServoSw = BServoHL ;//接下来要进入闭环，准备舵机切换值
        LServoSw = LServoHL ;
        RServoSw = RServoHL ;
        //PServoSwitch = PServoHL ;//米帅添加
        return;
    }
    //printf("%f\n",CollectiveTmp);
    BServoHL = BServoSw + B_direction * CollectiveTmp ;//计算舵机PWM
    LServoHL = LServoSw + L_direction * CollectiveTmp ;
    RServoHL = RServoSw + R_direction * CollectiveTmp ; 
    //PServoHL=  PServoSwitch + P_direction * CollectiveTmp*3; //米帅添加
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
//        TakeOffVel = 0;//学长代码为UltrasoundNewHeightVel[0];参考速度从当前速度开始增加
//    }
//	if(IsAccelerateRise)//加速上升阶段
//    {
//        if(TakeOffVel < MaxHeightVel)//速度小于最大速度，速度慢慢增加
//        {
//            TakeOffVel += TakeOffAcc * SystemPeriod;
//        }
//        else
//        {
//            IsAccelerateRise = 0;//加速当最大速度后，状态改为减速上升
//        }
//    }
//	 else//减速上升阶段
//    {
//        if(TakeOffVel > 0)//速度大于零时，速度慢慢减小 
//        {
//            TakeOffVel -= TakeOffAcc * SystemPeriod;
//        }
//        else
//        {
//            TakeOffVel = 0;
//            AutoriseFlag2 = 0;//速度减到零后，将 上升的的高度加到 stage
//            HeightTrackStage += TakeOffHeight * 100;
//            
//            //自主起降完成后切为GPS控制
//            OperateSelect = OperateSelect | 0x0400;//GPS标志置1
//            OperateSelect2 = OperateSelect2 & 0xfffb;//MsAccHeightCtr标志置0
//            UltrasoundCtr = 0;
//            GpsHeightCtr = 1;
//            MsAccHeightCtr = 0;
//            CurrentHeightOffset = CurrentHeight[1] - GpsAccHeight[0]*1000;
//            //自主起降完成后开始执行航迹
//            if(TotalTraceNum > 0 && InRoute == 0)//如果之前已经收到了航迹参数，但尚未执行且现在不在执行航迹
//            {
//                DestStaticPoint();//真正算出前两条航迹（飞往指定经纬度点）的参数
//                InRoute = 1;//航迹飞行中，拒绝接收任何路径参数
//                RouteStop = 0;
//                CurrentTraceNum = 0; //重置当前正执行的轨迹
//                CurrentRouteFinish = 1;//准备开始第1条轨迹的初始化
//            }
//        }
//    }
//    TakeOffHeight += TakeOffVel * SystemPeriod;
}
