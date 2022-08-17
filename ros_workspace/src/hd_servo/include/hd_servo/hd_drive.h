/*定义手部驱动*/
#ifndef HD_DRIVE_H
#define HD_DRIVE_H
#include<math.h>
#include<iostream>
#include"SCServo.h"
#include"matrix.h"
#include"hd_kinematics.h"
#include"system_time.h"

//////test and calibration//////////////////////////////////
// #define test //定义test 测试夹爪 运动速度低
// #define pos_calibration //第一次安装需要校准中位，此后不再需要
//////////////////////////////////////////////////////////////

#define Finger_L 0
#define Finger_R 1
#define SteptoDeg (360.0/4096.0)
#define DegtoStep (4096.0/360.0)
/*此转换针对初始位置校准时，两手指平行垂直于基座的情况，此时舵机初始角均为180.0°*/
#define theta_1_calibration 270.0//用于将关节角校准为发送给舵机的指令关节角度
#define theta_2_calibration 180.0//指令关节角度（°）还需要转换为步数，
#define theta_3_calibration 180.0//关节角->舵机角转换公式：舵机角=theta_x_calibration-theta_X

using namespace std;
/*手部驱动类*/
class hd_drive
{
private:
    SMSBL sms_;
    double StartTime_=0.0;

    // const s16 MotorCmd_LimtLow_[3]={1900,1000,1010};//关节安全限位
    // const s16 MotorCmd_LimtHig_[3]={3180,3120,3170};

    const s16 MotorCmd_LimtLow_[3]={0,0,0};//取消关节安全限位
    const s16 MotorCmd_LimtHig_[3]={3600,3600,3600};

    u8 ID_L_[3]={1,2,3};
    u8 ID_R_[3]={4,5,6};
    u8 IDN_=3;
    
    #ifndef test
        u16 Speed_[3]={800,800,800};//test为了安全，速度设置较低；{800,800,800};//°,需要转换成步数
        // u16 Speed_[3]={500,500,500};
    #else
        // u16 Speed_[3]={30,30,30};
        u16 Speed_[3]={300,300,300};//调试值

    #endif

    
    u8 ACC_[3]={50,50,50};
    // u8 ACC_[3]={100,100,100};  //加速度大常常导致较大的冲击


public:
    hd_drive(int finger);
    ~hd_drive();

    void Motor_feedback();
    void Finger_init();
    void Finger_end();
    void Cmd_publish(MATRIX_D);
    friend  void SetCurtimeasStartTime(hd_drive &finger);
    friend double GetOffsetTime(hd_drive &finger);


    int flag_;
    // MATRIX_D Goal_Pos;
    // MATRIX_D Origin_Pos;
    MATRIX_D Goal_Pos=MatD31(0.0,0.0,0.0);
    MATRIX_D Origin_Pos=MatD31(0.0,0.0,0.0);

    int Motor_Load[3];
    int Motor_Speed[3];
    int Motor_Temper[3];
    int Motor_Pos[3];
    int Motor_Voltage[3];
    int Motor_Current[3];

    double Joint_theta[3];
};



#endif