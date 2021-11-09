#include "define.h"

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

extern void DriveTrainPID(float setpoint);
extern void ArcMove(float turn, float smallarcD);
extern void TurnPID(int setpoint);
extern void TurnTestPID(float setpoint);
extern void SmallLiftPID(void* setpoint);
extern void SmallLiftPIDA(float setppoint);
extern void BigLiftPID(void* setpoint);

//Auton select
extern void OnLeftButton();
extern void OnCenterButton();
extern void OnRightButton();

extern double RateLimiter(double input, int lastCall, double maxRateChange, double prevOutput);

#endif
