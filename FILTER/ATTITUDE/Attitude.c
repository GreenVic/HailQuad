#include "Attitude.h"



short aacx,aacy,aacz;	        		
short gyrox,gyroy,gyroz;        		
s16 mx,my,mz;							
short compOrigin[3],compOrigin1[3];		        
float BodyAngleRate[3],BodyAcc[3][3];		
float EarthAngleRate[3][3],EarthAcc[3][3];
float AngleRate[2][3],Angle[2][3];		
float AngleHmr[2],AngleAcc[2][2];		

float GyroOrigin[3];    
float AccOrigin[3];		
float Mag[3];				
float Mag_Max[3];			
float Mag_Min[3];
float Angle_Accorigin[3][3];  
float Cali_Sum_Gyro[3];    
float Cali_Sum_Angle[3];	
float Cali_Sum_Acc[3];
float Average_Gyro[3];	
float Average_Acc[3];
float Zero_Angle[3];			
int Cali_Length = 100;     		
long Cali_Num = 0; 
float HX,HY;


//����Ϊ�������ʹ��
/*
corrected_point = C * (raw_point - center)

where corrected_point, raw_point, center are 3x1 vectors (x,y,z) and C = evecs*diag(1./radii)*evecs' is a symmetric 3x3 matrix.

If you'd rather have the points mapped onto a sphere of radius k (rather than unit sphere), then use k*C in the above instead of just C.*/


float Ellipsoid_Center[3] = {-57.358,-94.727,-7.9114};
float Ellipsoid_Radius[3] = {545.57,535.21,441.97};	//�����Ƹ�����������ĺͰ뾶��������ϣ���ƽ�Ʋ��������Ų���
float Ellipsoid_Evecs[3][3] = {{0.70381,-0.70753,0.063718},  
								{-0.6728,0.63509,-0.37947},
								{-0.22801,0.30994,0.92301}};

float C[3][3] = {{    0.0019,    0.0000,    0.0001},    //C = evecs*diag(1./radii)*evecs'
				 {    0.0000,    0.0019,   	0.0001},
				 {    0.0001,    0.0001,    0.0022}};




double SafeSin(double tempnum)//Ϊ�˷�ֹsin��cos��Ĳ�������,���ƽǶȵľ���ֵС�ڵ���pi/2
{
    double resultnum;

    if (tempnum >= 1.5707596)
    {
        resultnum = 1.5707596;
    }
    else if (tempnum <= -1.5707596)
    {
        resultnum = -1.5707596;
    }
    else
    {
        resultnum = tempnum;
    }
    return resultnum;
}

float SafeAsin(float tempnum)//ʹ��asin�����ı�������(-1,1)����ֹ�˲����ַ�ɢ
{
    float resultnum;

    if (tempnum >= 1)
    {
        resultnum = 1;
    }
    else if (tempnum <= -1)
    {
        resultnum = -1;
    }
    else
    {
        resultnum = tempnum;
    }
    
    return resultnum;
}

void ReadMPU9250()//�Ӿ��ᴫ����MPU9250�ж�ȡ���ٶȡ��Ǽ��ٶȺʹŸ�Ӧǿ��
{
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
	MPU_Get_Gyroscope(&gyroy,&gyrox,&gyroz);	//�õ�����������
	Get_HMC5983(&mx,&my,&mz);			//�õ�������������
//	printf("%d,%d,%d;\r\n",mx,my,mz);
	aacy = -aacy;
	gyrox = -gyrox;
	//����������״̬ ���Ϊ������
	GyroOrigin[0] = 500*gyrox/65536.0*DtoR;
	GyroOrigin[1] = 500*gyroy/65536.0*DtoR;   	
	GyroOrigin[2] = 500*gyroz/65536.0*DtoR;   	
	
	
	
	
	/*���¼��ٶȼ�״̬*/  
	AccOrigin[0] = 4*aacx/65536.0;
	AccOrigin[1] = 4*aacy/65536.0;   	
	AccOrigin[2] = 4*aacz/65536.0;   	
	
	
	/*���´�����״̬*/
//	Mag[0] = (4800*mx/65536.0 - Ellipsoid_Center[0])*(Ellipsoid_Evecs[0][0]*Ellipsoid_Evecs[0][0]/Ellipsoid_Radius[0]);
//	Mag[1] = (4800*my/65536.0 - Ellipsoid_Center[1])*(Ellipsoid_Evecs[1][1]*Ellipsoid_Evecs[1][1]/Ellipsoid_Radius[1]);
//	Mag[2] = (4800*mz/65536.0 - Ellipsoid_Center[2])*(Ellipsoid_Evecs[2][2]*Ellipsoid_Evecs[2][2]/Ellipsoid_Radius[2]);
//	Mag[0] = 4800*mx/65536.0;
//	Mag[1] = 4800*my/65536.0;
//	Mag[2] = 4800*mz/65536.0;

	Mag[0] = (4800*mx/65536.0 - Ellipsoid_Center[0]);
	Mag[1] = (4800*my/65536.0 - Ellipsoid_Center[1]);
	Mag[2] = (4800*mz/65536.0 - Ellipsoid_Center[2]);


	Mag[0] = Mag[0]*C[0][0] + Mag[1]*C[0][1] + Mag[2]*C[0][2];
	Mag[1] = Mag[0]*C[1][0] + Mag[1]*C[1][1] + Mag[2]*C[1][2];
	Mag[2] = Mag[0]*C[2][0] + Mag[1]*C[2][1] + Mag[2]*C[2][2];

	
	HX = Mag[0] * cos(Angle[0][1]) + Mag[1] * sin(Angle[0][0]) * sin(Angle[0][1]) - Mag[2] * sin(Angle[0][1]) * cos(Angle[0][0]);
    HY = Mag[1] * cos(Angle[0][0]) + Mag[2] * sin(Angle[0][0]);  
	AngleHmr[1]	= AngleHmr[0];
    AngleHmr[0] = -atan2(HY,HX);
//	printf("%f;\r\n",AngleHmr[0]/DtoR);
	/*���ٶȼƵó��Ƕ�,ֻ�ܵó������ͺ����*/
	AngleAcc[1][0] = AngleAcc[0][0];
	AngleAcc[1][1] = AngleAcc[0][1];
	AngleAcc[0][1] = -asin(SafeAsin(AccOrigin[1]));//������-Y������
	AngleAcc[0][0] = asin(SafeAsin(AccOrigin[0]/cos(AngleAcc[0][1])));//�����-X������ 
}

void SensorCalibration()//�������궨
{	
	
//	Average_Gyro[0] = 0;
//	Average_Gyro[1] = 0;
//	Average_Gyro[2] = 0;
//		
//	Zero_Angle[0] = 0;
//	Zero_Angle[1] = 0;
	while(Cali_Num<Cali_Length)
	{	
		if(Timer_Flag)
		{			
			
			ReadMPU9250();
			
			
			Cali_Sum_Gyro[0] += GyroOrigin[0];
			Cali_Sum_Gyro[1] += GyroOrigin[1];
			Cali_Sum_Gyro[2] += GyroOrigin[2];
			
			Cali_Sum_Acc[0] += AccOrigin[0];
			Cali_Sum_Acc[1] += AccOrigin[1];
			Cali_Sum_Acc[2] += AccOrigin[2];
			
			Cali_Sum_Angle[0] += AngleAcc[0][0];
			Cali_Sum_Angle[1] += AngleAcc[0][1];

			Cali_Num++;
			Timer_Flag = 0;
		}
	}

	Average_Gyro[0] = Cali_Sum_Gyro[0]/Cali_Length;
	Average_Gyro[1] = Cali_Sum_Gyro[1]/Cali_Length;
	Average_Gyro[2] = Cali_Sum_Gyro[2]/Cali_Length;
	
	Average_Acc[0] = Cali_Sum_Acc[0]/Cali_Length;
	Average_Acc[1] = Cali_Sum_Acc[1]/Cali_Length;
	Average_Acc[2] = Cali_Sum_Acc[2]/Cali_Length;
		
	Zero_Angle[0] = Cali_Sum_Angle[0]/Cali_Length;
	Zero_Angle[1] = Cali_Sum_Angle[1]/Cali_Length;
	
	
	//��ֵ
	Angle[0][0] = Zero_Angle[0];
	Angle[0][1] = Zero_Angle[1];
	EarthAcc[0][0] = Average_Acc[0];
	EarthAcc[0][1] = Average_Acc[1];
	EarthAcc[0][2] = Average_Acc[2];
	EarthAcc[1][0] = Average_Acc[0];
	EarthAcc[1][1] = Average_Acc[1];
	EarthAcc[1][2] = Average_Acc[2];
	
	HX = Mag[0] * cos(Angle[0][1]) + Mag[1] * sin(Angle[0][0]) * sin(Angle[0][1]) - Mag[2] * sin(Angle[0][1]) * cos(Angle[0][0]);
    HY = Mag[1] * cos(Angle[0][0]) + Mag[2] * sin(Angle[0][0]);    
    AngleHmr[0] =  -atan2(HY,HX);
	AngleHmr[1] = AngleHmr[0];
	
	
	Angle[0][2] = AngleHmr[0];
	Angle[1][2] = Angle[0][2];
   
	


}


void AttitudeFilter()//��̬�˲�
{
	
	ReadMPU9250();
	//printf("%f,%f		",GyroOrigin[0]/DtoR,GyroOrigin[1]/DtoR);
	//printf("%f,%f		",AccOrigin[0],AccOrigin[1]);
	
	
	
	BodyAngleRate[0] = GyroOrigin[0] - Average_Gyro[0];
	BodyAngleRate[1] = GyroOrigin[1] - Average_Gyro[1];
	BodyAngleRate[2] = -(GyroOrigin[2] - Average_Gyro[2]);
	
	
	BodyAcc[2][0] = BodyAcc[1][0];
	BodyAcc[2][1] = BodyAcc[1][1];
	BodyAcc[2][2] = BodyAcc[1][2];
	BodyAcc[1][0] = BodyAcc[0][0];
	BodyAcc[1][1] = BodyAcc[0][1];
	BodyAcc[1][2] = BodyAcc[0][2];
	BodyAcc[0][0] = AccOrigin[0];
	BodyAcc[0][1] = AccOrigin[1];
	BodyAcc[0][2] = AccOrigin[2];
	
	EarthAcc[2][0] = EarthAcc[1][0];
	EarthAcc[2][1] = EarthAcc[1][1];
	EarthAcc[2][2] = EarthAcc[1][2];
	EarthAcc[1][0] = EarthAcc[0][0];
	EarthAcc[1][1] = EarthAcc[0][1];
	EarthAcc[1][2] = EarthAcc[0][2];
	 /// ����
//	EarthAcc[0][0] = BodyAcc[0][0]*cos(SafeSin(Angle[0][1]))*cos(SafeSin(Angle[0][2])) - BodyAcc[0][1]*cos(SafeSin(Angle[0][1]))*sin(SafeSin(Angle[0][2])) \
//		+BodyAcc[0][2]*sin(SafeSin(Angle[0][1]));
//	EarthAcc[0][1] = BodyAcc[0][0]*(sin(SafeSin(Angle[0][0]))*sin(SafeSin(Angle[0][1]))*cos(SafeSin(Angle[0][2]))-cos(SafeSin(Angle[0][0]))*sin(SafeSin(Angle[0][2])))\
//		- BodyAcc[0][1]*(sin(SafeSin(Angle[0][0]))*sin(SafeSin(Angle[0][1]))*sin(SafeSin(Angle[0][2]))+cos(SafeSin(Angle[0][0]))*cos(SafeSin(Angle[0][2])))\
//		+BodyAcc[0][2]*sin(SafeSin(Angle[0][0]))*cos(SafeSin(Angle[0][1]));

//	EarthAcc[0][2] = -BodyAcc[0][0]*sin(Angle[0][1])+\
//		+ BodyAcc[0][1]*cos(Angle[0][1])*sin(Angle[0][0])+\
//		+ BodyAcc[0][2]*cos(Angle[0][0])*sin(Angle[0][1]);  
	EarthAcc[0][2] = BodyAcc[0][2]/(cos(SafeSin(Angle[0][0]))*cos(SafeSin(Angle[0][1])));


//	printf("%f,%f,",AngleAcc[0][0]/DtoR,AngleAcc[0][1]/DtoR);
	//����任
	EarthAngleRate[2][0] = EarthAngleRate[1][0];
	EarthAngleRate[2][1] = EarthAngleRate[1][1];
	EarthAngleRate[2][2] = EarthAngleRate[1][2];
	EarthAngleRate[1][0] = EarthAngleRate[0][0];
	EarthAngleRate[1][1] = EarthAngleRate[0][1];
	EarthAngleRate[1][2] = EarthAngleRate[0][2];
	EarthAngleRate[0][0] = BodyAngleRate[0] + BodyAngleRate[1] * sin(SafeSin(Angle[0][0])) * tan(SafeSin(Angle[0][1])) \
							+ BodyAngleRate[2] * cos(SafeSin(Angle[0][0])) * tan(SafeSin(Angle[0][1]));//���-X
	EarthAngleRate[0][1] = BodyAngleRate[1] * cos(SafeSin(Angle[0][0])) - BodyAngleRate[2] * sin(SafeSin(Angle[0][0]));//����-Y
	EarthAngleRate[0][2] = (BodyAngleRate[1] * sin(SafeSin(Angle[0][0])) + BodyAngleRate[2] * cos(SafeSin(Angle[0][0])))/cos(SafeSin(Angle[1][1]));//ƫ����-Z
	
	//�����ǰʱ��ƫ���ǳ�����һʱ��ƫ����0.8pi������Ϊ�Ƕ�Խ����180��-180���������ش���
	if(AngleHmr[0] - Angle[0][2] > 2.513f)AngleHmr[0] = AngleHmr[0] - 2 * pi; 
	if(AngleHmr[0] - Angle[0][2] < -2.513f)AngleHmr[0] = AngleHmr[0] + 2 * pi;
	
	//���㲹���˲���Ľ��ٶ�
	AngleRate[1][0] = AngleRate[0][0];
	AngleRate[1][1] = AngleRate[0][1];
	AngleRate[1][2] = AngleRate[0][2];
	AngleRate[0][0] = AngleRate[1][0] + EarthAngleRate[0][0] - EarthAngleRate[1][0] - (AttitudeFbP + \
							AttitudeFbI * SystemPeriod/2) * (Angle[0][0] - AngleAcc[0][0]) + \
							(AttitudeFbP - AttitudeFbI * SystemPeriod/2) * (Angle[1][0] - AngleAcc[1][0]);
	AngleRate[0][1] = AngleRate[1][1] + EarthAngleRate[0][1] - EarthAngleRate[1][1] - (AttitudeFbP + \
							AttitudeFbI * SystemPeriod/2) * (Angle[0][1] - AngleAcc[0][1]) + \
							(AttitudeFbP - AttitudeFbI * SystemPeriod/2) * (Angle[1][1] - AngleAcc[1][1]);
	AngleRate[0][2] = AngleRate[1][2] + EarthAngleRate[0][2] - EarthAngleRate[1][2] - (AttitudeFbP+ \
							AttitudeFbI * SystemPeriod/2) * (Angle[0][2] - AngleHmr[0]) + \
							(AttitudeFbP - AttitudeFbI * SystemPeriod/2) * (Angle[1][2] - AngleHmr[1]);  
							
	//���㲹���˲���ĽǶȣ����ý��ٶȻ��֣�
	Angle[1][0] = Angle[0][0];
	Angle[1][1] = Angle[0][1];
	Angle[1][2] = Angle[0][2];
	Angle[0][0] = Angle[1][0] + (AngleRate[0][0] + AngleRate[1][0]) * SystemPeriod / 2;
	Angle[0][1] = Angle[1][1] + (AngleRate[0][1] + AngleRate[1][1]) * SystemPeriod / 2;
	Angle[0][2] = Angle[1][2] + (AngleRate[0][2] + AngleRate[1][2]) * SystemPeriod / 2;
	
	if(Angle[0][2] > pi)
	{
		Angle[0][2] -=  2 * pi;
		Angle[1][2] -=  2 * pi;
		AngleHmr[0] -= 2 * pi;
		AngleHmr[1] -= 2 * pi;
	}
	if(Angle[0][2] < -pi)
	{
		Angle[0][2] +=  2 * pi;
		Angle[1][2] +=  2 * pi;
		AngleHmr[0] += 2 * pi;
		AngleHmr[1] += 2 * pi;
	}

	
}




