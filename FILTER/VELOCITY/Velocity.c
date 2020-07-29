#include "Velocity.h"



float Body_AccLength,Body_AccCross; //机身纵向\横向水平加速度
float Filter_AccLength1,Filter_AccCross1,Filter_AccLength2,Filter_AccCross2;     //通过一阶二阶滤波的机体加速度
float Earth_AccSN[3],Earth_AccEW[3];    //地面坐标系的加速度
float Filter_AccSN1,Filter_AccEW1,Filter_AccSN2,Filter_AccEW2;     //通过一阶二阶滤波的地面加速度
float AccFilterParam = 20;



float GPS_Yaw,Gps_Velocity;
float GPS_VelSN,GPS_VelEW;
float Earth_VelSN[3],Earth_VelEW[3];

float VelocitySN[3],VelocityEW[3];

void VelocityFilter(nmea_msg gps)//位置滤波子函数，IMU/GPS或光流
{

	
	//float AccSN[3],AccEW[3];
	
	
	////原始数据处理////
	
	
//	GPS_Yaw = gps.dir/10;  //偏航角，除以10为数据转换
//	Gps_Velocity = gps.velocity/10;
	GPS_Yaw = 0;
    Gps_Velocity = 0;
	
	GPS_VelEW = Gps_Velocity * sin(GPS_Yaw * DtoR)*1.852/3.6;
    GPS_VelSN = Gps_Velocity * cos(GPS_Yaw * DtoR)*1.852/3.6;
	
	
	
	////GPS速度处理/////
	/*速度补偿，GPS接收机位于尾杆上，机身旋转时GPS接收机检测到速度，但机身位置并没有发生变化*/
    Earth_VelSN[2] = Earth_VelSN[1];
    Earth_VelSN[1] = Earth_VelSN[0];
    Earth_VelEW[2] = Earth_VelEW[1];
    Earth_VelEW[1] = Earth_VelEW[0];
	
	Earth_VelSN[0] = GPS_VelSN - EarthAngleRate[0][2] * LENGTH * sin(Angle[0][2]);
    Earth_VelEW[0] = GPS_VelEW + EarthAngleRate[0][2] * LENGTH * cos(Angle[0][2]);   
	
	
	//////速度滤波///////
		
	/*角度补偿，得到机身横向和纵向的水平加速度*/
	Body_AccLength = - G * AccOrigin[1] - G * sin(SafeSin(Angle[0][1])); //纵向加速度
	Body_AccCross  = - G * AccOrigin[0] + G * sin(SafeSin(Angle[0][0])); //横向加速度
	Body_AccLength = Body_AccLength * cos(SafeSin(Angle[0][1])); 
	Body_AccCross  = Body_AccCross * cos(SafeSin(Angle[0][0])); 
	//printf("%f,%f,",Body_AccLength,Body_AccCross);
	
	 //加速度二阶惯性滤波   AccFilterParam / (s + AccFilterParam)  ///惯性即低通,单线性
    
    Filter_AccCross1 = SystemPeriod*AccFilterParam*Body_AccCross/(SystemPeriod*AccFilterParam + 1) + Filter_AccCross1/(SystemPeriod*AccFilterParam + 1);   ///单线性变换
    Filter_AccCross2 = SystemPeriod*AccFilterParam*Filter_AccCross1/(SystemPeriod*AccFilterParam + 1) + Filter_AccCross2/(SystemPeriod*AccFilterParam + 1);
    Filter_AccLength1 = SystemPeriod*AccFilterParam*Body_AccLength/(SystemPeriod*AccFilterParam + 1) + Filter_AccLength1/(SystemPeriod*AccFilterParam + 1);
    Filter_AccLength2 = SystemPeriod*AccFilterParam*Filter_AccLength1/(SystemPeriod*AccFilterParam + 1) + Filter_AccLength2/(SystemPeriod*AccFilterParam + 1);
//    AccHeightFilter1 = SystemPeriod*AccFilterParam*(AccOrigin[2] - Average_Acc[2])/(SystemPeriod*AccFilterParam + 1) + AccHeightFilter1/(SystemPeriod*AccFilterParam + 1);
//    AccHeightFilter2 = SystemPeriod*AccFilterParam*AccHeightFilter1/(SystemPeriod*AccFilterParam + 1) + AccHeightFilter2/(SystemPeriod*AccFilterParam + 1);   
   
	
	Body_AccCross = Filter_AccCross2;
	Body_AccLength = Filter_AccLength2;	
	Angle[0][2]=0;
	 /*坐标变换，将机体坐标系的水平加速度转换为地面坐标系下正东正北的加速度*/
    Earth_AccSN[2] = Earth_AccSN[1];
    Earth_AccEW[2] = Earth_AccEW[1]; 
    Earth_AccSN[1] = Earth_AccSN[0];
    Earth_AccEW[1] = Earth_AccEW[0]; 
    Earth_AccSN[0] = -Body_AccCross * sin(Angle[0][2]) + Body_AccLength * cos(Angle[0][2]);//南北，对应EarthAcc[0][0]
    Earth_AccEW[0] = Body_AccCross * cos(Angle[0][2]) + Body_AccLength * sin(Angle[0][2]); //东西，对应EarthAcc[0][1]
	//printf("%f,%f;\r\n",Earth_AccSN[0],Earth_AccEW[0]);
	
	
	//二阶滤波
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
	
	//加速度与GPS得到的速度融合滤波得到速度。框图见软件文档说明中的高度滤波
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
