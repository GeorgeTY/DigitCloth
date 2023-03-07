
#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H

#include <stdio.h>
#include <time.h>

#define NSEC_PER_SEC	(1000000000)


extern double GetCurrentTime();
extern void ResetTime();
extern void ResetTimerCounter();
extern void TimeCounterAdd();
extern int GetTimerCounter();
extern double GetOffsetTime();
extern void SetStartTime(double time);
extern void SetCurtimeasStartTime();

#endif
