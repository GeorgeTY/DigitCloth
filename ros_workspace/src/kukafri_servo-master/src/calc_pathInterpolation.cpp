#include "calc_pathInterpolation.h"

// ステップ関数による軌道補間  
double CalcStepTraje(double orig, double goal) //一步到位，不插值
{
    return goal;
}

// 1 次関数による軌道補間
double Calc1JiTraje(double orig, double goal, double duration, double time)
{
    double ref = goal;
    double time_n = time/duration;//time 当前周期 和 轨迹规划开始周期 之间的时间差值，基本等于n*0.008s

    if(time_n <= 1)
        ref = orig + (goal-orig)*time_n;
    return ref;
}
double Calc1JiTraje(double orig, double goal, double duration, double time, bool& flag)
{
    double ref = goal;
    double time_n = time/duration;//time 当前周期 和 轨迹规划开始周期 之间的时间差值，基本等于n*0.008s

    if(time_n <= 1)
        {ref = orig + (goal-orig)*time_n;}
    else {flag=true;}
    return ref;
}

// sin 関数による軌道補間
double CalcSinTraje(double orig, double goal, double duration, double time)
{
    double ref = goal;
    double time_n = time/duration;

    if(time_n <= 1)
        ref = orig + (goal-orig)* time_n
            - (goal-orig) * sin(2.0*M_PI*time_n)/(2.0*M_PI);
    return ref;
}

double CalcSinTraje(double orig, double goal, double duration, double time, bool& flag)
{
    double ref = goal;
    double time_n = time/duration;

    if(time_n <= 1){
        ref = orig + (goal-orig)* time_n
            - (goal-orig) * sin(2.0*M_PI*time_n)/(2.0*M_PI);}
    else{flag=true;}
    
    return ref;
}

MATRIX_D CalcSlerp(double* startPos, double* cmdPos, double duration, double time, bool& flag)
{
    // printf("1"); 
    MATRIX_D refpos(7,1,cmdPos);
    // printf("2");
    // memcpy(refpos,cmdPos,7*sizeof(double));
    // printf("3");
    double time_n=time/duration;
    // printf("4");
    if (time_n>1)
    {
        flag=true;
        return refpos;
    }
    // printf("1");
    for (int i = 0; i < 3; i++)
    {
        refpos(i+1,1)=Calc1JiTraje(startPos[i],cmdPos[i],duration,time);
    }
    // printf("5");
    double startqua[4],cmdqua[4];
    memcpy(startqua,startPos+3,4*sizeof(double));
    memcpy(cmdqua,cmdPos+3,4*sizeof(double));
    // for (int i = 0; i < 4; i++)
    //   {
    //      printf("startqua[%d]:%f ",i,startqua[i]);
    //   }
    // for (int i = 0; i < 4; i++)
    //   {
    //      printf("cmdqua[%d]:%f ",i,cmdqua[i]);
    //   }
    double cosa=startqua[0]*cmdqua[0]+startqua[1]*cmdqua[1]+startqua[2]*cmdqua[2]+startqua[3]*cmdqua[3];
    // printf("\ncosa=%f",cosa);
    if (cosa<0)
    {
        for (int i = 0; i < 4; i++)
        {
            cmdqua[i]=-cmdqua[i];
        }
        cosa=-cosa;
    }
    // printf("8");
    double k0,k1;
    if (cosa>0.9995)
    {
        k0=1.0-time_n;
        k1=time_n;
    }
    else
    {
        // printf("cosa<=0.9995,start");
        double sina=sqrt(1.0-cosa*cosa);
        double a=atan2(sina,cosa);
        // printf("\nsina=%f, a=%f",sina,a);
        k0=sin((1.0-time_n)*a)/sina;
        k1=sin(time_n*a)/sina;
    }
    // printf("\nk0=%f ,k1=%f",k0,k1);
    for (int i = 3; i < 7; i++)
    {
        refpos(i+1,1)=startqua[i-3]*k0+cmdqua[i-3]*k1;
    }
    // refpos.print();
    return refpos;
    
    
}
