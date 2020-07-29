#include "AttitudeControl.h"
#include "SBUS.h"
int AngCtrFirst=0;
float AttTrackFilterAng[3];
float AngleDelta[3][3];
float AngleRate_Gyro[3][3];		//单由陀螺仪经低通滤波得到的角速度
float AngleRate_Delta[3][3];   //角速度的差值

//控制器输出
float ControlPitch;
float ControlRoll;
float ControlYaw;
float ControlCollective;

//控制器输入
float ControlSN = 0;//目标姿态角（地面坐标系）
float ControlEW = 0;
float ControlSnBody = 0;
float ControlEwBody = 0;

//姿态角零点
float PitchZero = 0;//直升机的实际零点
float RollZero = 0;
float YawZero = 0;
float PitchTargetZero;//进入控制器的零点
float RollTargetZero;
float YawTargetZero = 0;

//低通滤波器参数
float Filter_Gyro_F = 20;

//控制器参数
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
#define AttitudeBound 0.69813*0.75    //内环控制30度

unsigned int ServoHL[6] ;//接收机捕获值，代表接收机PWM信号的高电平时间，1000~2000ms
//遥控器给定的舵机输入指令，范围为1000~2000ms，对应角度为-90~90度，以下为遥控器指令
unsigned int BServoHL;//后舵机
unsigned int LServoHL;//左舵机
unsigned int RServoHL;//右舵机
unsigned int TServoHL;//尾舵机
unsigned int PServoHL;//油门舵机





void AttitudeControl()
{         
    float pitch_down_bound,pitch_up_bound,roll_down_bound,roll_up_bound,yaw_down_bound,yaw_up_bound;//内环控制器的输入界。角度*10
    int k;//Yaw是360的多少倍

    // 手动切自动时第一次进入内环控制程序
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
	//目标姿态角
    /*进入控制器的横滚俯仰目标零点,弧度：零点+外环输入+增稳量*/
    ControlSnBody = ControlEW * sin(Angle[0][2]) + ControlSN * cos(Angle[0][2]); 
    ControlEwBody = ControlEW * cos(Angle[0][2]) - ControlSN * sin(Angle[0][2]);
    RollTargetZero = RollZero + ControlEwBody;    //向右走为正
    PitchTargetZero = PitchZero - ControlSnBody;//向北走需要飞机低头，低头角度为负	
	
	
	
	///以下为实验舵机程序是否正确//
	RollTargetZero = (Remoter_Channel[3] - Cali_Channel[3])/2500.0f;  //当前接受值-标定初值    292-1604   1300对应角度0.52358（30°）
    PitchTargetZero = (-1)*(Remoter_Channel[1] - Cali_Channel[1])/2500.0f;
	YawTargetZero = (Remoter_Channel[0] - Cali_Channel[0])/1000.0f;   
//	printf("%10f,%10f,%10f;\r\n",RollTargetZero,PitchTargetZero,YawTargetZero);     
    //进入控制器的输入量整理   
    /*控制器的输入界*/
    pitch_down_bound = -AttitudeBound;
    pitch_up_bound = AttitudeBound;
    roll_down_bound = -AttitudeBound;
    roll_up_bound = AttitudeBound;
    yaw_down_bound = -AttitudeBound;
    yaw_up_bound = AttitudeBound;
    /*控制器的输入：目标零点-当前值*/
    AngleDelta[2][2] = AngleDelta[1][2];
    AngleDelta[2][1] = AngleDelta[1][1];
    AngleDelta[2][0] = AngleDelta[1][0];    
    AngleDelta[1][2] = AngleDelta[0][2];
    AngleDelta[1][1] = AngleDelta[0][1];
    AngleDelta[1][0] = AngleDelta[0][0];   
    AngleDelta[0][2] = YawTargetZero - Angle[0][2];     
    AngleDelta[0][1] = PitchTargetZero - Angle[0][1];
    AngleDelta[0][0] = RollTargetZero - Angle[0][0];   
	
	//角速度进行低通滤波
	AngleRate_Gyro[2][2] = AngleRate_Gyro[1][2]; 
    AngleRate_Gyro[1][2] = AngleRate_Gyro[0][2]; 
    AngleRate_Gyro[0][2] = ((EarthAngleRate[0][2]+EarthAngleRate[1][2])*Filter_Gyro_F*SystemPeriod/2 - AngleRate_Gyro[1][2]*(Filter_Gyro_F*SystemPeriod/2-1))/(1+Filter_Gyro_F*SystemPeriod/2);    
//	printf("%10f,%10f;\r\n", AngleRate_Gyro[0][2],AngleRate[0][2]);


	AngleRate_Delta[0][2] = YawTargetZero - AngleRate_Gyro[0][2];

    //针对偏航跟踪的单独处理
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
    //YawD传的为开始积分的误差界  YawF传的为积分结果的界限
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
	
    //俯仰轴控制 
    ControlPitch = (2 - PitchF * SystemPeriod) * ControlPitch + AngleDelta[0][1] * (PitchP * PitchF * SystemPeriod + 2 * PitchP + 2* PitchF * PitchD) + AngleDelta[1][1] * (PitchP * PitchF * SystemPeriod - 2 * PitchP - 2* PitchF * PitchD);
    ControlPitch = ControlPitch/(2 + PitchF * SystemPeriod);	
    //横滚轴控制 
    ControlRoll = (2 - RollF * SystemPeriod) * ControlRoll + AngleDelta[0][0] * (RollP * RollF * SystemPeriod + 2 * RollP + 2* RollF * RollD) + AngleDelta[1][0] * (RollP * RollF * SystemPeriod - 2 * RollP - 2* RollF * RollD);
    ControlRoll = ControlRoll/(2 + RollF * SystemPeriod);
    //偏航轴PD控制      
//  ControlYaw = AngleDelta[0][2] * YawP + yaw_integrate * YawI;    
	ControlYaw = AngleRate_Delta[0][2] * YawP/4;
//	printf("%10f,%10f,%10f;\r\n",ControlPitch,ControlRoll,ControlYaw);     
    //限幅
    if(ControlRoll > 28000)ControlRoll = 28000;
    if(ControlRoll < -28000)ControlRoll = -28000;
    if(ControlPitch > 11500)ControlPitch = 12000;
    if(ControlPitch < -11500)ControlPitch = -12000;
    if(ControlYaw > 11500)ControlYaw = 15000;
    if(ControlYaw < -14300)ControlYaw = -15000;
	//printf("%f,    %f,    %f\r\n",AngleDelta[0][1],AngleDelta[0][2],AngleDelta[0][3]);
	//printf("%f,    %f,    %f\r\n\r\n",ControlRoll,ControlPitch,ControlYaw);
	
	



}


