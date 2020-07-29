#include "Position.h"


float PositionSN[3],PositionEW[3];
float BodyPosSN[3],BodyPosEW[3];
void PositionFilter(void)
{
	PositionSN[2] = PositionSN[1];
    PositionSN[1] = PositionSN[0];
	PositionEW[2] = PositionEW[1]; 
    PositionEW[1] = PositionEW[0];
	
	PositionSN[0] = 2 * VelocitySN[0] * SystemPeriod - 2 * VelocitySN[2] * SystemPeriod + \
	(PositionSNFbI*SystemPeriod*SystemPeriod+2*PositionSNFbP*SystemPeriod)* BodyPosSN[0] + \
	(PositionSNFbI*SystemPeriod*SystemPeriod-2*PositionSNFbP*SystemPeriod) * BodyPosSN[2] + 2*PositionSNFbI*SystemPeriod*SystemPeriod * BodyPosSN[1];
    
	PositionSN[0] = (PositionSN[0] - (2*PositionSNFbI*SystemPeriod*SystemPeriod-8) * PositionSN[1] - \
	(PositionSNFbI*SystemPeriod*SystemPeriod-2*PositionSNFbP*SystemPeriod+4) * PositionSN[2])/(PositionSNFbI*SystemPeriod*SystemPeriod+2*PositionSNFbP*SystemPeriod+4); 
	
	PositionEW[0] = 2 * VelocityEW[0] * SystemPeriod - 2 * VelocityEW[2] * SystemPeriod +\
	(PositionEWFbI*SystemPeriod*SystemPeriod+2*PositionEWFbP*SystemPeriod) * BodyPosEW[0] +\
	(PositionEWFbI*SystemPeriod*SystemPeriod-2*PositionEWFbP*SystemPeriod) * BodyPosEW[2] + 2*PositionEWFbI*SystemPeriod*SystemPeriod * BodyPosEW[1];
   
    PositionEW[0] = (PositionEW[0] - (2*PositionEWFbI*SystemPeriod*SystemPeriod-8) * PositionEW[1] - \
	(PositionEWFbI*SystemPeriod*SystemPeriod-2*PositionEWFbP*SystemPeriod+4) * PositionEW[2])/(PositionEWFbI*SystemPeriod*SystemPeriod+2*PositionEWFbP*SystemPeriod+4);   
}
