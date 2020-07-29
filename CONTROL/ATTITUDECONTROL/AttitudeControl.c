#include "AttitudeControl.h"
#include "SBUS.h"
int AngCtrFirst=0;
float AttTrackFilterAng[3];
float AngleDelta[3][3];
float AngleRate_Gyro[3][3];		//���������Ǿ���ͨ�˲��õ��Ľ��ٶ�
float AngleRate_Delta[3][3];   //���ٶȵĲ�ֵ

//���������
float ControlPitch;
float ControlRoll;
float ControlYaw;
float ControlCollective;

//����������
float ControlSN = 0;//Ŀ����̬�ǣ���������ϵ��
float ControlEW = 0;
float ControlSnBody = 0;
float ControlEwBody = 0;

//��̬�����
float PitchZero = 0;//ֱ������ʵ�����
float RollZero = 0;
float YawZero = 0;
float PitchTargetZero;//��������������
float RollTargetZero;
float YawTargetZero = 0;

//��ͨ�˲�������
float Filter_Gyro_F = 20;

//����������
float PitchP = 500 * DtoR;
float PitchI =0;
float PitchD = 50 * DtoR;
int PitchF = 40;
float RollP = 500 * DtoR;
float RollI =0;
float RollD = 50 * DtoR;
int RollF = 40;
int YawP = 1000 * DtoR;
int YawI = 0;
int YawD;
int YawF;
float yaw_integrate = 1;
float yaw_error_bound = 20;
float yaw_integrate_bound = 100;
#define AttitudeBound 0.69813*0.75    //�ڻ�����30��

unsigned int ServoHL[6] ;//���ջ�����ֵ��������ջ�PWM�źŵĸߵ�ƽʱ�䣬1000~2000ms
//ң���������Ķ������ָ���ΧΪ1000~2000ms����Ӧ�Ƕ�Ϊ-90~90�ȣ�����Ϊң����ָ��
unsigned int BServoHL;//����
unsigned int LServoHL;//����
unsigned int RServoHL;//�Ҷ��
unsigned int TServoHL;//β���
unsigned int PServoHL;//���Ŷ��





void AttitudeControl()
{         
    float pitch_down_bound,pitch_up_bound,roll_down_bound,roll_up_bound,yaw_down_bound,yaw_up_bound;//�ڻ�������������硣�Ƕ�*10
    int k;//Yaw��360�Ķ��ٱ�

    // �ֶ����Զ�ʱ��һ�ν����ڻ����Ƴ���
    if(AngCtrFirst == 0)
    {
		AngCtrFirst = 1;
		
        AttTrackFilterAng[0] = 0.0;
        AttTrackFilterAng[1] = 0.0;
        AttTrackFilterAng[2] = 0.0;
            
        AngleDelta[0][2] = 0;
        AngleDelta[0][1] = 0;
        AngleDelta[0][0] = 0;
        AngleDelta[1][2] = 0;
        AngleDelta[1][1] = 0;
        AngleDelta[1][0] = 0;
        AngleDelta[2][2] = 0;
        AngleDelta[2][1] = 0;
        AngleDelta[2][0] = 0;
       
        ControlPitch=0.0; 
        ControlRoll = 0.0; 
        ControlYaw=0.0;                    
    }
    
	ReceiveOrder();
	//Ŀ����̬��
    /*����������ĺ������Ŀ�����,���ȣ����+�⻷����+������*/
    ControlSnBody = ControlEW * sin(Angle[0][2]) + ControlSN * cos(Angle[0][2]); 
    ControlEwBody = ControlEW * cos(Angle[0][2]) - ControlSN * sin(Angle[0][2]);
    RollTargetZero = RollZero + ControlEwBody;    //������Ϊ��
    PitchTargetZero = PitchZero - ControlSnBody;//������Ҫ�ɻ���ͷ����ͷ�Ƕ�Ϊ��	
	
	
	
	///����Ϊʵ���������Ƿ���ȷ//
	RollTargetZero = (Remoter_Channel[3] - Cali_Channel[3])/2500.0f;  //��ǰ����ֵ-�궨��ֵ    292-1604   1300��Ӧ�Ƕ�0.52358��30�㣩
    PitchTargetZero = (-1)*(Remoter_Channel[1] - Cali_Channel[1])/2500.0f;
	YawTargetZero = (Remoter_Channel[0] - Cali_Channel[0])/1000.0f;   
//	printf("%10f,%10f,%10f;\r\n",RollTargetZero,PitchTargetZero,YawTargetZero);     
    //���������������������   
    /*�������������*/
    pitch_down_bound = -AttitudeBound;
    pitch_up_bound = AttitudeBound;
    roll_down_bound = -AttitudeBound;
    roll_up_bound = AttitudeBound;
    yaw_down_bound = -AttitudeBound;
    yaw_up_bound = AttitudeBound;
    /*�����������룺Ŀ�����-��ǰֵ*/
    AngleDelta[2][2] = AngleDelta[1][2];
    AngleDelta[2][1] = AngleDelta[1][1];
    AngleDelta[2][0] = AngleDelta[1][0];    
    AngleDelta[1][2] = AngleDelta[0][2];
    AngleDelta[1][1] = AngleDelta[0][1];
    AngleDelta[1][0] = AngleDelta[0][0];   
    AngleDelta[0][2] = YawTargetZero - Angle[0][2];     
    AngleDelta[0][1] = PitchTargetZero - Angle[0][1];
    AngleDelta[0][0] = RollTargetZero - Angle[0][0];   
	
	//���ٶȽ��е�ͨ�˲�
	AngleRate_Gyro[2][2] = AngleRate_Gyro[1][2]; 
    AngleRate_Gyro[1][2] = AngleRate_Gyro[0][2]; 
    AngleRate_Gyro[0][2] = ((EarthAngleRate[0][2]+EarthAngleRate[1][2])*Filter_Gyro_F*SystemPeriod/2 - AngleRate_Gyro[1][2]*(Filter_Gyro_F*SystemPeriod/2-1))/(1+Filter_Gyro_F*SystemPeriod/2);    
//	printf("%10f,%10f;\r\n", AngleRate_Gyro[0][2],AngleRate[0][2]);


	AngleRate_Delta[0][2] = YawTargetZero - AngleRate_Gyro[0][2];

    //���ƫ�����ٵĵ�������
    if((AngleDelta[0][2]-AngleDelta[1][2])>2)
         AngleDelta[0][2]=AngleDelta[0][2]-6.28f;
    else if((AngleDelta[0][2]-AngleDelta[1][2])<-2)
         AngleDelta[0][2]=AngleDelta[0][2]+6.28f;    
    if(AngCtrFirst == 0)
    {
        AngCtrFirst = 1;
        AngleDelta[2][1] = AngleDelta[0][1];
        AngleDelta[2][0] = AngleDelta[0][0];  
        AngleDelta[1][1] = AngleDelta[0][1];
        AngleDelta[1][0] = AngleDelta[0][0];
        ControlPitch = AngleDelta[0][1] * PitchP;
        ControlRoll = AngleDelta[0][0] * RollP;
    }
    if(AngleDelta[0][2] > 4.7f)AngleDelta[0][2] = AngleDelta[0][2] - 2 * pi;// 4.7 = 1.5 * pi
    if(AngleDelta[0][2] < -4.7f)AngleDelta[0][2] = AngleDelta[0][2] + 2 * pi;// 4.7 = 1.5 * pi   
    //YawD����Ϊ��ʼ���ֵ�����  YawF����Ϊ���ֽ���Ľ���
    if(fabs(AngleDelta[0][2]) < yaw_error_bound)
        yaw_integrate += AngleDelta[0][2] * SystemPeriod;
    if(yaw_integrate > yaw_integrate_bound)
        yaw_integrate = yaw_integrate_bound;
    else if(yaw_integrate < -yaw_integrate_bound)
        yaw_integrate = -yaw_integrate_bound; 
    if(AngleDelta[0][1] < pitch_down_bound)AngleDelta[0][1] = pitch_down_bound;
    if(AngleDelta[0][1] > pitch_up_bound)AngleDelta[0][1] = pitch_up_bound;
    if(AngleDelta[0][0] < roll_down_bound)AngleDelta[0][0] = roll_down_bound;
    if(AngleDelta[0][0] > roll_up_bound)AngleDelta[0][0] = roll_up_bound;   
    if(AngleDelta[0][2] < yaw_down_bound)AngleDelta[0][2] = yaw_down_bound;   
    if(AngleDelta[0][2] > yaw_up_bound)AngleDelta[0][2] = yaw_up_bound; 

	if(AngleRate_Delta[0][2] < yaw_down_bound)AngleDelta[0][2] = yaw_down_bound;   
    if(AngleRate_Delta[0][2] > yaw_up_bound)AngleDelta[0][2] = yaw_up_bound; 
	
    //��������� 
    ControlPitch = (2 - PitchF * SystemPeriod) * ControlPitch + AngleDelta[0][1] * (PitchP * PitchF * SystemPeriod + 2 * PitchP + 2* PitchF * PitchD) + AngleDelta[1][1] * (PitchP * PitchF * SystemPeriod - 2 * PitchP - 2* PitchF * PitchD);
    ControlPitch = ControlPitch/(2 + PitchF * SystemPeriod);	
    //�������� 
    ControlRoll = (2 - RollF * SystemPeriod) * ControlRoll + AngleDelta[0][0] * (RollP * RollF * SystemPeriod + 2 * RollP + 2* RollF * RollD) + AngleDelta[1][0] * (RollP * RollF * SystemPeriod - 2 * RollP - 2* RollF * RollD);
    ControlRoll = ControlRoll/(2 + RollF * SystemPeriod);
    //ƫ����PD����      
//  ControlYaw = AngleDelta[0][2] * YawP + yaw_integrate * YawI;    
	ControlYaw = AngleRate_Delta[0][2] * YawP/4;
//	printf("%10f,%10f,%10f;\r\n",ControlPitch,ControlRoll,ControlYaw);     
    //�޷�
    if(ControlRoll > 28000)ControlRoll = 28000;
    if(ControlRoll < -28000)ControlRoll = -28000;
    if(ControlPitch > 11500)ControlPitch = 12000;
    if(ControlPitch < -11500)ControlPitch = -12000;
    if(ControlYaw > 11500)ControlYaw = 15000;
    if(ControlYaw < -14300)ControlYaw = -15000;
	//printf("%f,    %f,    %f\r\n",AngleDelta[0][1],AngleDelta[0][2],AngleDelta[0][3]);
	//printf("%f,    %f,    %f\r\n\r\n",ControlRoll,ControlPitch,ControlYaw);
	
	



}


