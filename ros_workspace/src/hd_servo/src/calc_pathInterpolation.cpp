#include "calc_pathInterpolation.h"


/*阶跃插值*/
double CalcStepTraje(double orig, double goal) //一步到位，不插值
{
    return goal;
}
/* 一次路径插值*/
double Calc1JiTraje(double orig, double goal, double duration, double time)
{
    double ref = goal;
    double time_n = time/duration;//time 当前周期 和 轨迹规划开始周期 之间的时间差值，基本等于n*0.008s

    if(time_n <= 1)
        ref = orig + (goal-orig)*time_n;
    return ref;
}
/*sin轨迹插值*/
double CalcSinTraje(double orig, double goal, double duration, double time)
{
    double ref = goal;
    double time_n = time/duration;

    if(time_n <= 1)
        ref = orig + (goal-orig)* time_n
            - (goal-orig) * sin(2.0*M_PI*time_n)/(2.0*M_PI);
    return ref;
}