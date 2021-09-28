#include "define.h"

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

extern void SmallLiftPID(void* setpoint);
extern void BigLiftPID(void* setpoint);

extern double RateLimiter(double input, int lastCall, double maxRateChange, double prevOutput);

#endif
