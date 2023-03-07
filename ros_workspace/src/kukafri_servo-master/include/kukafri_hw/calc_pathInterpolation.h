#ifndef PATH_INTERPOLATION_H
#define PATH_INTERPOLATION_H

#include<math.h>
#include<string.h>
// #include<cstdio>

#include "matrix.h"

#define INTERPOLATION_ONE 0
#define INTERPOLATION_STEP 1
#define INTERPOLATION_SIN 2

extern double CalcStepTraje(double orig, double goal);
extern double Calc1JiTraje(double orig, double goal, double duration, double time);
extern double Calc1JiTraje(double orig, double goal, double duration, double time, bool& flag);

extern double CalcSinTraje(double orig, double goal, double duration, double time);
extern double CalcSinTraje(double orig, double goal, double duration, double time,bool& flag);

extern MATRIX_D CalcSlerp(double* startPos, double* cmdPos, double duration, double time, bool& flag);

#endif
