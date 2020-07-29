#ifndef TAKEOFFLAND_H_
#define TAKEOFFLAND_H_
#include "global.h"


extern int AutoriseFlag2;
extern float TakeOffHeight;
extern float TakeOffVel;
extern float LandHeight;
extern float LandVel;
extern int IsDescend;
extern int IsOpenLoop;
extern int IsFirstEnterOpenLoop;
extern float ControlRollOpenLoop;
extern float ControlPitchOpenLoop;
extern unsigned int BServoHL;//����
extern unsigned int LServoHL;//����
extern unsigned int RServoHL;//�Ҷ����������AttitudeControl
extern unsigned int BServoSw;
extern unsigned int LServoSw;
extern unsigned int RServoSw;//������HeightControl
void AutoTakeOff(void);
void AutoTakeOffLandInit(void);
void GenerateTakeOffHeightVel(void);
void AutoLand(void);



#endif /*TAKEOFFLAND_H_*/
