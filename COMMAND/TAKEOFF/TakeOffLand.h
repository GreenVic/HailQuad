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
extern unsigned int BServoHL;//后舵机
extern unsigned int LServoHL;//左舵机
extern unsigned int RServoHL;//右舵机，定义在AttitudeControl
extern unsigned int BServoSw;
extern unsigned int LServoSw;
extern unsigned int RServoSw;//定义在HeightControl
void AutoTakeOff(void);
void AutoTakeOffLandInit(void);
void GenerateTakeOffHeightVel(void);
void AutoLand(void);



#endif /*TAKEOFFLAND_H_*/
