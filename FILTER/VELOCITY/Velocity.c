#include "Velocity.h"



float Body_AccLength,Body_AccCross; //��������\����ˮƽ���ٶ�
float Filter_AccLength1,Filter_AccCross1,Filter_AccLength2,Filter_AccCross2;     //ͨ��һ�׶����˲��Ļ�����ٶ�
float Earth_AccSN[3],Earth_AccEW[3];    //��������ϵ�ļ��ٶ�
float Filter_AccSN1,Filter_AccEW1,Filter_AccSN2,Filter_AccEW2;     //ͨ��һ�׶����˲��ĵ�����ٶ�
float AccFilterParam = 20;



float GPS_Yaw,Gps_Velocity;
float GPS_VelSN,GPS_VelEW;
float Earth_VelSN[3],Earth_VelEW[3];

float VelocitySN[3],VelocityEW[3];

void VelocityFilter(nmea_msg gps)//λ���˲��Ӻ�����IMU/GPS�����
{

	
	//float AccSN[3],AccEW[3];
	
	
	////ԭʼ���ݴ���////
	
	
//	GPS_Yaw = gps.dir/10;  //ƫ���ǣ�����10Ϊ����ת��
//	Gps_Velocity = gps.velocity/10;
	GPS_Yaw = 0;
    Gps_Velocity = 0;
	
	GPS_VelEW = Gps_Velocity * sin(GPS_Yaw * DtoR)*1.852/3.6;
    GPS_VelSN = Gps_Velocity * cos(GPS_Yaw * DtoR)*1.852/3.6;
	
	
	
	////GPS�ٶȴ���/////
	/*�ٶȲ�����GPS���ջ�λ��β���ϣ�������תʱGPS���ջ���⵽�ٶȣ�������λ�ò�û�з����仯*/
    Earth_VelSN[2] = Earth_VelSN[1];
    Earth_VelSN[1] = Earth_VelSN[0];
    Earth_VelEW[2] = Earth_VelEW[1];
    Earth_VelEW[1] = Earth_VelEW[0];
	
	Earth_VelSN[0] = GPS_VelSN - EarthAngleRate[0][2] * LENGTH * sin(Angle[0][2]);
    Earth_VelEW[0] = GPS_VelEW + EarthAngleRate[0][2] * LENGTH * cos(Angle[0][2]);   
	
	
	//////�ٶ��˲�///////
		
	/*�ǶȲ������õ��������������ˮƽ���ٶ�*/
	Body_AccLength = - G * AccOrigin[1] - G * sin(SafeSin(Angle[0][1])); //������ٶ�
	Body_AccCross  = - G * AccOrigin[0] + G * sin(SafeSin(Angle[0][0])); //������ٶ�
	Body_AccLength = Body_AccLength * cos(SafeSin(Angle[0][1])); 
	Body_AccCross  = Body_AccCross * cos(SafeSin(Angle[0][0])); 
	//printf("%f,%f,",Body_AccLength,Body_AccCross);
	
	 //���ٶȶ��׹����˲�   AccFilterParam / (s + AccFilterParam)  ///���Լ���ͨ,������
    
    Filter_AccCross1 = SystemPeriod*AccFilterParam*Body_AccCross/(SystemPeriod*AccFilterParam + 1) + Filter_AccCross1/(SystemPeriod*AccFilterParam + 1);   ///�����Ա任
    Filter_AccCross2 = SystemPeriod*AccFilterParam*Filter_AccCross1/(SystemPeriod*AccFilterParam + 1) + Filter_AccCross2/(SystemPeriod*AccFilterParam + 1);
    Filter_AccLength1 = SystemPeriod*AccFilterParam*Body_AccLength/(SystemPeriod*AccFilterParam + 1) + Filter_AccLength1/(SystemPeriod*AccFilterParam + 1);
    Filter_AccLength2 = SystemPeriod*AccFilterParam*Filter_AccLength1/(SystemPeriod*AccFilterParam + 1) + Filter_AccLength2/(SystemPeriod*AccFilterParam + 1);
//    AccHeightFilter1 = SystemPeriod*AccFilterParam*(AccOrigin[2] - Average_Acc[2])/(SystemPeriod*AccFilterParam + 1) + AccHeightFilter1/(SystemPeriod*AccFilterParam + 1);
//    AccHeightFilter2 = SystemPeriod*AccFilterParam*AccHeightFilter1/(SystemPeriod*AccFilterParam + 1) + AccHeightFilter2/(SystemPeriod*AccFilterParam + 1);   
   
	
	Body_AccCross = Filter_AccCross2;
	Body_AccLength = Filter_AccLength2;	
	Angle[0][2]=0;
	 /*����任������������ϵ��ˮƽ���ٶ�ת��Ϊ��������ϵ�����������ļ��ٶ�*/
    Earth_AccSN[2] = Earth_AccSN[1];
    Earth_AccEW[2] = Earth_AccEW[1]; 
    Earth_AccSN[1] = Earth_AccSN[0];
    Earth_AccEW[1] = Earth_AccEW[0]; 
    Earth_AccSN[0] = -Body_AccCross * sin(Angle[0][2]) + Body_AccLength * cos(Angle[0][2]);//�ϱ�����ӦEarthAcc[0][0]
    Earth_AccEW[0] = Body_AccCross * cos(Angle[0][2]) + Body_AccLength * sin(Angle[0][2]); //��������ӦEarthAcc[0][1]
	//printf("%f,%f;\r\n",Earth_AccSN[0],Earth_AccEW[0]);
	
	
	//�����˲�
//	Filter_AccSN1 = SystemPeriod*AccFilterParam*Earth_AccSN[0]/(SystemPeriod*AccFilterParam + 1) + Filter_AccSN1/(SystemPeriod*AccFilterParam + 1);
//    Filter_AccSN2 = SystemPeriod*AccFilterParam*Filter_AccSN1/(SystemPeriod*AccFilterParam + 1) + Filter_AccSN2/(SystemPeriod*AccFilterParam + 1);
//    Filter_AccEW1 = SystemPeriod*AccFilterParam*Earth_AccEW[0]/(SystemPeriod*AccFilterParam + 1) + Filter_AccEW1/(SystemPeriod*AccFilterParam + 1);
//    Filter_AccEW2 = SystemPeriod*AccFilterParam*Filter_AccEW1/(SystemPeriod*AccFilterParam + 1) + Filter_AccEW2/(SystemPeriod*AccFilterParam + 1);
//	
//	printf("%f,%f,",Filter_AccSN2,Filter_AccEW2);
//    
	VelocitySN[2] = VelocitySN[1];
    VelocitySN[1] = VelocitySN[0];	
	VelocityEW[2] = VelocityEW[1]; 
    VelocityEW[1] = VelocityEW[0];
	
	//���ٶ���GPS�õ����ٶ��ں��˲��õ��ٶȡ���ͼ������ĵ�˵���еĸ߶��˲�
    VelocitySN[0] = 2 * Earth_AccSN[0] * SystemPeriod - 2 * Earth_AccSN[2] * SystemPeriod + \
	(VelocitySNFbI*SystemPeriod*SystemPeriod+2*VelocitySNFbP*SystemPeriod)* Earth_VelSN[0] + \
	(VelocitySNFbI*SystemPeriod*SystemPeriod-2*VelocitySNFbP*SystemPeriod) * Earth_VelSN[2] + 2*VelocitySNFbI*SystemPeriod*SystemPeriod * Earth_VelSN[1];
    
	VelocitySN[0] = (VelocitySN[0] - (2*VelocitySNFbI*SystemPeriod*SystemPeriod-8) * VelocitySN[1] - \
	(VelocitySNFbI*SystemPeriod*SystemPeriod-2*VelocitySNFbP*SystemPeriod+4) * VelocitySN[2])/(VelocitySNFbI*SystemPeriod*SystemPeriod+2*VelocitySNFbP*SystemPeriod+4); 
	
	VelocityEW[0] = 2 * Earth_AccEW[0] * SystemPeriod - 2 * Earth_AccEW[2] * SystemPeriod +\
	(VelocityEWFbI*SystemPeriod*SystemPeriod+2*VelocityEWFbP*SystemPeriod) * Earth_VelEW[0] +\
	(VelocityEWFbI*SystemPeriod*SystemPeriod-2*VelocityEWFbP*SystemPeriod) * Earth_VelEW[2] + 2*VelocityEWFbI*SystemPeriod*SystemPeriod * Earth_VelEW[1];
   
    VelocityEW[0] = (VelocityEW[0] - (2*VelocityEWFbI*SystemPeriod*SystemPeriod-8) * VelocityEW[1] - \
	(VelocityEWFbI*SystemPeriod*SystemPeriod-2*VelocityEWFbP*SystemPeriod+4) * VelocityEW[2])/(VelocityEWFbI*SystemPeriod*SystemPeriod+2*VelocityEWFbP*SystemPeriod+4);                  
	//printf("%f,%f;\r\n",VelocitySN[0],VelocityEW[0]);
}
