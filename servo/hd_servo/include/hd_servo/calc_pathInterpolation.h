#ifndef PATH_INTERPOLATION_H
#define PATH_INTERPOLATION_H

#include<math.h>
#include<string.h>
#include "matrix.h"

extern double CalcStepTraje(double orig, double goal);
extern double Calc1JiTraje(double orig, double goal, double duration, double time);
extern double CalcSinTraje(double orig, double goal, double duration, double time);

#endif
