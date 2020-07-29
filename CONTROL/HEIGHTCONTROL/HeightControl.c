#include "TakeoffLand.h"
#include "HeightControl.h"

void HeightControl(void)
{
	if(HighCtrFirst == 0) //第一次进入此函数
    {
        HighCtrFirst = 1; 
        /*以切换瞬间的值为切换后的零点*/
        SwitchHeight = Height[0] * 1000 * GpsHeightCtr + CurrentHeightOffset;//mm，与学长代码相比去掉了气压计和超声波得到高度的选项，只使用GPS和加速度计得到高度
        if(AutoriseFlag == 2)//如果是自主起飞结束第一次进入普通高度控制
        {
            AutoriseFlag = 0;
            //HeightDelta[0] = HeightDeltaWtd[0];

            //TheliState &= 0xf0;
            //TheliState |= 0x02;
        }
        else
        {                         
            ControlHeight[0] = 0.0;
            HeightDelta[0] = 0.0;
        } 
    }

    //汪雄峰
    /*目标高度*/
//    HeightTrackWhole = HeightTrackStage + HeightTrack[0] * HeightTrackFlag;//汪雄峰不知道为什么要修改为下面的 
    HeightTrackWhole = HeightTrackStage + HeightTrack[0] * 1;//attention!!! BY ZJ


    TargetHeight[2] =TargetHeight[1];
    TargetHeight[1] =TargetHeight[0];
    TargetHeightVel[1] = TargetHeightVel[0];
    if(is_remote_move_heli)
    {
        TargetHeight[0] = SwitchHeight + HeightTrackWhole * 10+remote_move_height_stage * 1000; //mm
        TargetHeightVel[0] = RouteVelHeight[0] + remote_move_up_vel[0] * 100;
    }
    else if(AutoriseFlag2 == 1)
    {
        GenerateTakeOffHeightVel();
        TargetHeight[0] = SwitchHeight + TakeOffHeight * 1000;
        TargetHeightVel[0] = TakeOffVel * 100;
    }
    else if(IsDescend)
    {
        AutoLand();
        TargetHeight[0] = SwitchHeight + HeightTrackWhole * 10 - LandHeight*1000; //mm
        TargetHeightVel[0] = - LandVel * 100;  //cm
    }
    else
    {
        TargetHeight[0] = SwitchHeight + HeightTrackWhole * 10; //mm
        TargetHeightVel[0] = RouteVelHeight[0];  //cm
        //RouteVelHeight[0] = 0;
    }
    //printf("%f\n",TargetHeight[0]);
    //printf("%f %d %f\n",UltrasoundAccHeight[0],SwitchHeight,TargetHeight[0]);
    
    //printf("%d\t%f\t%f\t%f\n",HeightTrack[0],HeightTrackStage,HeightTrackWhole,TargetHeight[0]);
    //printf("%f %f %f %f\n",remote_move_up_vel[0],remote_move_height_stage,TargetHeight[0],TargetHeightVel[0]);
    /*当前高度*/
//    CurrentHeight = UltrasoundHeight[0] * UltrasoundCtr + GpsHeight * GpsHeightCtr + MsAccHeight[0] * MsAccHeightCtr * 1000;
    //这段移到positionfilter.c里了
//    CurrentHeight_GPS1[2]=CurrentHeight_GPS1[1];
//    CurrentHeight_GPS1[1]=CurrentHeight_GPS1[0];
//    CurrentHeight_GPS1[0]=RadarAccHeight[0] * 1000 * UltrasoundCtr + Height_GPS1[0] * 1000 * GpsHeightCtr + MsAccHeight[0] * MsAccHeightCtr * 1000;//mm
//    
//    CurrentHeight_GPS2[2]=CurrentHeight_GPS2[1];
//    CurrentHeight_GPS2[1]=CurrentHeight_GPS2[0];
//    CurrentHeight_GPS2[0]=RadarAccHeight[0] * 1000 * UltrasoundCtr + Height_GPS2[0] * 1000 * GpsHeightCtr + MsAccHeight[0] * MsAccHeightCtr * 1000;//mm
    
    /*控制器的输入量：目标高度-当前高度*/
//    HeightDelta[1] = (TargetHeight[1]+Offset_TargetHeight[1] - CurrentHeight[1]) * 0.1;
    HeightDelta[1] = HeightDelta[0];
    if(GpsHeightCtr==1)//gps控制时才加gps的切换补偿量，其他方式控制时不加
        HeightDelta[0] = (TargetHeight[0]+Offset_TargetHeight[0] - CurrentHeight[0]) * 0.1f;//cm
    else
        HeightDelta[0] = (TargetHeight[0] - CurrentHeight[0]) * 0.1f;//cm     
    //printf("%d %f\n",CurrentHeight[0],TargetHeight[0]);
    //printf("%d  %f %d %d\n",CurrentHeight[0] , CurrentHeightOffset,CurrentHeight_GPS1[0],CurrentHeight_GPS2[0]);
    //汪雄峰
    //DeltaHeightHealth();
    
      
    if(HeightDelta[0] < -HeightBound)HeightDelta[0] = -HeightBound;
    if(HeightDelta[0] > HeightBound)HeightDelta[0] = HeightBound;
    if(HeightDelta[1] < -HeightBound)HeightDelta[1] = -HeightBound;
    if(HeightDelta[1] > HeightBound)HeightDelta[1] = HeightBound;
    
    if(HeightDelta[0] < 50 && HeightDelta[0] > -50) //误差在0.5m以内才开始积分，BY ZJ
    {
        SumHeightDelta = SumHeightDelta + HeightDelta[0];//cm, BY ZJ
//        SumHeightDelta = SumHeightDelta + 0.1 * (TargetHeight - RadarAccHeight[0] * 1000 * UltrasoundCtr + GpsHeight * GpsHeightCtr + MsAccHeight[0] * MsAccHeightCtr * 1000);
        if( SumHeightDelta > 100 || SumHeightDelta < -100 )
        {
            if( SumHeightDelta > 0 )
                SumHeightDelta = 100;
            else
                SumHeightDelta = -100;
        }//积分环节上限，BY ZJ
    }
//    printf("%d\t", GpsHeightCtr);
//    printf("%d\t", MsAccHeightCtr);
//    printf("%d\t", TargetHeight);
//    printf("%d\t", CurrentHeight);
//    printf("%f(cm)  ", HeightDelta[0]);
//    printf("%f(cm)\n", SumHeightDelta);


//        ControlHeight = ControlHeight * 1000;
//        ControlHeight = HeightP * 0.1 * (TargetHeight - RadarAccHeight[0] * 1000 * UltrasoundCtr + GpsHeight * GpsHeightCtr + MsAccHeight[0] * MsAccHeightCtr * 1000) + HeightD * (HeightDelta[0] - HeightDelta[1])/SystemPeriod + HeightF * SystemPeriod * SumHeightDelta;
//        printf("ControlHeihgt %f\n", ControlHeight);
        //BY ZJ，底下一行是8月23日实验之后修改的，控制的P和I用未经过融合的激光雷达测量值，D仍用融合后的值

	HeightDeltaVel[1]=HeightDeltaVel[0];
	HeightDeltaVel[0]= VelocityH[0] * 100 - TargetHeightVel[0]+Offset_VelTargetHeight[0];
	if(HeightDelta[0] < 50 && HeightDelta[0] > -50)
	{
		SumHeightDelta = SumHeightDelta + HeightDelta[0];
	}
	if(HeightDeltaVel[0]>100 && HeightDeltaVel[0] <-100)
	{
		if( HeightDeltaVel[0] > 0 )
			HeightDeltaVel[0] = 100;
		else
			HeightDeltaVel[0] = -100;            
	}
	if( SumHeightDelta > 150 || SumHeightDelta < -150 )
	{
		if( SumHeightDelta > 0 )
			SumHeightDelta = 150;
		else
			SumHeightDelta = -150;
	}
	//以下四行注释于141109，目的在于向高度引入速度反馈。BY ZJ
//        ControlHeight[3] =ControlHeight[2];
//        ControlHeight[2] =ControlHeight[1];
//        ControlHeight[1] =ControlHeight[0];
//        ControlHeight[0] = (2 - GpsHeightF * SystemPeriod) * ControlHeight[1] + (2 * GpsHeightP + 2 * GpsHeightD * GpsHeightF + GpsHeightP * GpsHeightF * SystemPeriod) *   HeightDelta[0] + (GpsHeightP * GpsHeightF * SystemPeriod - 2 * GpsHeightP - 2 * GpsHeightD * GpsHeightF) * HeightDelta[1] ;
//        ControlHeight[0] = ControlHeight[0]/(2 + GpsHeightF * SystemPeriod);    
	ControlHeight[3] =ControlHeight[2];        
	ControlHeight[2] =ControlHeight[1];
	ControlHeight[1] =ControlHeight[0];
	ControlHeight[0] =GpsHeightP * HeightDelta[0] + GpsHeightD * (-1) * HeightDeltaVel[0];  
		
    //printf(" %f ",ControlHeight[0]);
    //printf("%f %d %f %f\n",TargetHeight[0],CurrentHeight[0],HeightDelta[0],ControlHeight[0]);
    //printf("%f  %f  %f  %d  %f  %f  %f\n",Offset_TargetHeight[0],TargetHeight[0],HeightDelta[0],CurrentHeight[0],HeightDeltaVel[0],SumHeightDelta,ControlHeight[0]);
//    printf("%f    %f\n",VelBFiltered,SumHeightDelta);
}
