#include "define.h"

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

extern void DriveTrainPID(float setpoint);
extern void WallPush(int time);
extern void PlatformGrab();
extern void ArcMove(float left, float right, turnDirection direction);
extern void CoarseTurn(int setpoint);
extern void FineTurn(int setpoint, float kP = 170, float kI = 1.5);
extern void PTurn(int setpoint);
extern void RingIntake(int rotations);
extern void RingGrab();
extern void Lift(liftState state);

//Auton select
extern void OnLeftButton();
extern void OnCenterButton();
extern void OnRightButton();

extern double RateLimiter(double input, int lastCall, double maxRateChange, double prevOutput);

#endif
