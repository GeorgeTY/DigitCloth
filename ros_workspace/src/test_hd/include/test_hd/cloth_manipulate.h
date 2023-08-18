#ifndef CLOTH_MANIPULATE
#define CLOTH_MANIPULATE

#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <numeric>
#include <vector>
#include <string>
#include "hd_servo/EndPos.h"
#include "get_height/Height.h"
#include "get_height/Odometer_result.h"
#include "test_hd/state.h"

// #define COUNT_MODE 10//总计运动十次，用于测量移动距离
// #define no_force_control
#define Modify_digit_sensor
#define PI 3.14159265
#define Radius 35.0
#define deg2rad (PI / 360.0)
#define rad2deg (360.0 / PI)
#define InitL_PosX -32.0 //-28.0
#define InitR_PosX -8.0  // 2.0
#define InitL_PosY 135.0
#define InitR_PosY 130.0
#define InitL_PosAngle 0.0
#define InitR_PosAngle 0.0
#define Modify 0.065
#define frequency 15
#define Target_height 12000 // 0.5
#define Theta_threshold 50.0
#define w_deg 5.0
#define X_Threshold (0 + 10)

using namespace std;

class Move_cloth
{
private:
    enum flag
    {
        init_position = 0,
        force_control,
        init_Pos_after_force,
        turnL_clock /*通过摩擦面带动布料向手内*/,
        moveL_up, /*moveL_up与turnL_anticlock配合，完成接触区域的转换*/
        moveL_down,
        turnL_anticlock, /**/
        stop
    };
    flag flag_;

    float deta_theta;
    const float w = w_deg;
    ros::Duration init_duration;
    ros::Time init_begin;

    const float target_height = Target_height;
    const float theta_len = 10.0;
    const float theta_miu = (theta_len / Radius) * rad2deg;
    const float theta_threshold = Theta_threshold; // 接触面为高摩擦区对应的弧度角
    float deta_ythreshold;
    float start_angle;

    /*力相关*/
    float sum_height;
    float upArea_height;
    float downArea_height;
    float init_sum_height; // 静态噪声值
    const float KP = 0.2;
    const float KI = 0.0;
    const float KD = 0.2;
    float E1;
    float E2;
    float E3;

public:
    int count; // 统计操作次数

private:
    void action1(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float deta_theta);
    void action2(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float deta_theta);
    void action3(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float deta_theta);
    void action4(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float deta_theta);

public:
    Move_cloth(){};
    ~Move_cloth(){};
    void Init(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R);
    void Set_MoveDistance_in(float distance, hd_servo::EndPos &msg_L);
    void Move_Cloth_in(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, get_height::Height &current_height, get_height::Odometer_result &odometer_result);

    void Set_MoveDistance_out(float distance, hd_servo::EndPos &msg_L);
    void Move_Cloth_out(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, get_height::Height &current_height);
    string get_HdControl_state();
};

#endif