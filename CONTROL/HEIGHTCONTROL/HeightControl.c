#include "TakeoffLand.h"
#include "HeightControl.h"

void HeightControl(void)
{
	if(HighCtrFirst == 0) //��һ�ν���˺���
    {
        HighCtrFirst = 1; 
        /*���л�˲���ֵΪ�л�������*/
        SwitchHeight = Height[0] * 1000 * GpsHeightCtr + CurrentHeightOffset;//mm����ѧ���������ȥ������ѹ�ƺͳ������õ��߶ȵ�ѡ�ֻʹ��GPS�ͼ��ٶȼƵõ��߶�
        if(AutoriseFlag == 2)//�����������ɽ�����һ�ν�����ͨ�߶ȿ���
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

    //���۷�
    /*Ŀ��߶�*/
//    HeightTrackWhole = HeightTrackStage + HeightTrack[0] * HeightTrackFlag;//���۷岻֪��ΪʲôҪ�޸�Ϊ����� 
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
    /*��ǰ�߶�*/
//    CurrentHeight = UltrasoundHeight[0] * UltrasoundCtr + GpsHeight * GpsHeightCtr + MsAccHeight[0] * MsAccHeightCtr * 1000;
    //����Ƶ�positionfilter.c����
//    CurrentHeight_GPS1[2]=CurrentHeight_GPS1[1];
//    CurrentHeight_GPS1[1]=CurrentHeight_GPS1[0];
//    CurrentHeight_GPS1[0]=RadarAccHeight[0] * 1000 * UltrasoundCtr + Height_GPS1[0] * 1000 * GpsHeightCtr + MsAccHeight[0] * MsAccHeightCtr * 1000;//mm
//    
//    CurrentHeight_GPS2[2]=CurrentHeight_GPS2[1];
//    CurrentHeight_GPS2[1]=CurrentHeight_GPS2[0];
//    CurrentHeight_GPS2[0]=RadarAccHeight[0] * 1000 * UltrasoundCtr + Height_GPS2[0] * 1000 * GpsHeightCtr + MsAccHeight[0] * MsAccHeightCtr * 1000;//mm
    
    /*����������������Ŀ��߶�-��ǰ�߶�*/
//    HeightDelta[1] = (TargetHeight[1]+Offset_TargetHeight[1] - CurrentHeight[1]) * 0.1;
    HeightDelta[1] = HeightDelta[0];
    if(GpsHeightCtr==1)//gps����ʱ�ż�gps���л���������������ʽ����ʱ����
        HeightDelta[0] = (TargetHeight[0]+Offset_TargetHeight[0] - CurrentHeight[0]) * 0.1f;//cm
    else
        HeightDelta[0] = (TargetHeight[0] - CurrentHeight[0]) * 0.1f;//cm     
    //printf("%d %f\n",CurrentHeight[0],TargetHeight[0]);
    //printf("%d  %f %d %d\n",CurrentHeight[0] , CurrentHeightOffset,CurrentHeight_GPS1[0],CurrentHeight_GPS2[0]);
    //���۷�
    //DeltaHeightHealth();
    
      
    if(HeightDelta[0] < -HeightBound)HeightDelta[0] = -HeightBound;
    if(HeightDelta[0] > HeightBound)HeightDelta[0] = HeightBound;
    if(HeightDelta[1] < -HeightBound)HeightDelta[1] = -HeightBound;
    if(HeightDelta[1] > HeightBound)HeightDelta[1] = HeightBound;
    
    if(HeightDelta[0] < 50 && HeightDelta[0] > -50) //�����0.5m���ڲſ�ʼ���֣�BY ZJ
    {
        SumHeightDelta = SumHeightDelta + HeightDelta[0];//cm, BY ZJ
//        SumHeightDelta = SumHeightDelta + 0.1 * (TargetHeight - RadarAccHeight[0] * 1000 * UltrasoundCtr + GpsHeight * GpsHeightCtr + MsAccHeight[0] * MsAccHeightCtr * 1000);
        if( SumHeightDelta > 100 || SumHeightDelta < -100 )
        {
            if( SumHeightDelta > 0 )
                SumHeightDelta = 100;
            else
                SumHeightDelta = -100;
        }//���ֻ������ޣ�BY ZJ
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
        //BY ZJ������һ����8��23��ʵ��֮���޸ĵģ����Ƶ�P��I��δ�����ںϵļ����״����ֵ��D�����ںϺ��ֵ

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
	//��������ע����141109��Ŀ��������߶������ٶȷ�����BY ZJ
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
