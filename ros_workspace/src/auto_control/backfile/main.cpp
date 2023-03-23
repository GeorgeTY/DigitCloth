#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <numeric>
#include "hd_servo/EndPos.h"
#include "get_height/Height.h"

using namespace std;
#define PI 3.14159265
#define deg2rad (PI / 360.0)
#define rad2deg (360.0 / PI)
#define InitL_PosX -35.0
#define InitR_PosX 0.0
#define InitL_PosY 135.0
#define InitR_PosY 135.0
#define InitL_PosAngle 0.0
#define InitR_PosAngle 0.0

enum flag
{
    init_position = 0,
    force_control,
    turnL_clock /*通过摩擦面带动布料向手内*/,
    moveL_up,        /*moveL_up与turnL_anticlock配合，完成接触区域的转换*/
    turnL_anticlock, /**/
};

void Digit_callback(const get_height::Height::ConstPtr &msg, get_height::Height &current_height)
{
    ROS_INFO("Callback: get digit info !");
    current_height = *msg;
    return;
}

vector<float> Calculate_SubHeight(const get_height::Height &current_height)
{
    vector<float> SubHeight(6, 0.0); // 将传感器数据初步分为6个区数据,传感器数据为6*3
    SubHeight[0] = current_height.height_data0[0] + current_height.height_data0[1] + current_height.height_data0[2];
    SubHeight[1] = current_height.height_data1[0] + current_height.height_data1[1] + current_height.height_data1[2];
    SubHeight[2] = current_height.height_data2[0] + current_height.height_data2[1] + current_height.height_data2[2];
    SubHeight[3] = current_height.height_data3[0] + current_height.height_data3[1] + current_height.height_data3[2];
    SubHeight[4] = current_height.height_data4[0] + current_height.height_data4[1] + current_height.height_data4[2];
    SubHeight[5] = current_height.height_data5[0] + current_height.height_data5[1] + current_height.height_data5[2];
    // ROS_INFO("Sub0: %lf",SubHeight[0]);
    // ROS_INFO("Sub1: %lf",SubHeight[1]);
    // ROS_INFO("Sub2: %lf",SubHeight[2]);
    // ROS_INFO("Sub3: %lf",SubHeight[3]);
    // ROS_INFO("Sub4: %lf",SubHeight[4]);
    // ROS_INFO("Sub5: %lf",SubHeight[5]);

    return SubHeight;
}
void action1(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float deta_theta)
{
    /*两种情况
    1：V0_L=0,wR!=0,V0_R=0 接触点相对digit位置不变，接触点处两表面相对滑动；
    2：V0=wR=V0_R!=0 接触点相对digit位置不变，接触点处两表面相对滑动；
    以下采用情况1。
    */
    float deta_y = deta_theta * deg2rad * 35;
    msg_L.Z_Angle = msg_L.Z_Angle + deta_theta;
    return;
}
void action2(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float deta_theta)
{
    /*V0_L=wR!=0,V_R=0;接触点相对Digit上移，接触点处两表面相对静止*/
    float deta_y = deta_theta * deg2rad * 35;
    msg_L.Z_Angle = msg_L.Z_Angle + deta_theta;
    msg_L.Y_Axis = msg_L.Y_Axis + deta_y;
    return;
}
void action3(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float deta_theta)
{
    /*两种情况
    1：V0_L=0,wR!=0,V0_R=0 接触点相对digit位置不变，接触点处两表面相对滑动；
    2：V0=wR=V0_R!=0 接触点相对digit位置不变，接触点处两表面相对滑动；
    以下采用情况2。
    */
    float deta_y = deta_theta * deg2rad * 35;
    msg_L.Z_Angle = msg_L.Z_Angle + deta_theta;
    msg_L.Y_Axis = msg_L.Y_Axis + deta_y;
    msg_R.Y_Axis = msg_R.Y_Axis + deta_y;
    return;
}
void action4(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R, float deta_theta)
{
    /*wR=0,V0_L!=0,V0_R=0;接触点相对于digit上移*/
    float deta_y = deta_theta * deg2rad * 35;
    msg_L.Y_Axis = msg_L.Y_Axis + deta_y;
    return;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_hd");
    ros::NodeHandle nh;
    ros::Publisher CmdL_pub = nh.advertise<hd_servo::EndPos>("Goal_EndPos_L", 1);
    ros::Publisher CmdR_pub = nh.advertise<hd_servo::EndPos>("Goal_EndPos_R", 1); // 当前策略只需要发布一次右侧手位置
    hd_servo::EndPos msg_L;
    hd_servo::EndPos msg_R;
    msg_L.X_Axis = InitL_PosX;
    msg_L.Y_Axis = InitL_PosY;
    msg_L.Z_Angle = InitL_PosAngle;
    msg_R.X_Axis = InitR_PosX;
    msg_R.Y_Axis = InitR_PosY;
    msg_R.Z_Angle = InitR_PosAngle;

    get_height::Height current_height;
    /*Ldigit_height现被安装在Finger_R*/
    ros::Subscriber DigitL_sub = nh.subscribe<get_height::Height>("/Ldigit_height", 1, boost::bind(&Digit_callback, _1, ref(current_height)));
    float sum_height;
    float upArea_height;
    float downArea_height;
    const float target_height = 1.5;
    const float theta_miu = 54.0; // 接触面为高摩擦区对应的弧度角

    const float KP = 0.2;
    const float KI = 0.0;

    const float KD = 0.2;

    float E1 = 0;
    float E2 = 0;
    float E3 = 0;

    flag flag_ = init_position;
    ros::Duration init_duration(6);
    ros::Time init_begin = ros::Time::now();

    const int w = 8.0; // 两度每s
    int frequency = 12;
    int flag_dirORtrans = 0;
    ros::Rate loop_rate(frequency); // 频率高会导致只有单手能收到信号 不知道与什么有关
    while (ros::ok())
    {
        /* enum flag{
            init_position=0,
            force_control,
            turnL_clock，//通过摩擦面带动布料向手内
            moveL_up,//moveL_up与turnL_anticlock配合，完成接触区域的转换
            turnL_anticlock,
         };*/

        /*
        1:第一阶段：运动到初始居中位置
        2：第二阶段：开始进行力控，digit只进行x方向力控
        */
        float deta_theta = w * (1 / ((float)frequency)); // 单位度/秒

        /*digit力反馈情况*/
        vector<float> sub_height = Calculate_SubHeight(current_height);
        sum_height = accumulate(sub_height.begin(), sub_height.end(), 0.0);
        upArea_height = accumulate(sub_height.begin(), sub_height.begin() + 4, 0.0);
        downArea_height = accumulate(sub_height.begin() + 4, sub_height.end(), 0.0);
        ROS_INFO("Sum_force: %lf", sum_height);
        // ROS_INFO("Up_force: %lf",upArea_height);
        // ROS_INFO("Down_force: %lf",downArea_height);

        E1 = E2;
        E2 = E3;
        E3 = target_height - sum_height;
        float deta_height = KP * E3 + KI * (E1 + E2 + E3) + KD * ((E3 - E2) - (E2 - E1));
        deta_height = abs(deta_height) > 1.0 ? (deta_height / abs(deta_height) * 1.0) : deta_height;

        switch (flag_)
        {
        case init_position:
            if ((ros::Time::now() - init_begin) > init_duration)
                flag_ = force_control;
            break;
        case force_control:
            if (sum_height < target_height)
            {
                /*力控*/
                msg_R.X_Axis = msg_R.X_Axis - deta_height;
            }
            else
            {
                flag_ = turnL_clock;
            }
            flag_ = turnL_clock;
            break;
        case turnL_clock:
            action1(msg_L, msg_R, deta_theta);

            if (msg_L.Z_Angle > (theta_miu / 2))
                flag_ = moveL_up;

            ROS_INFO("turnL_clock");
            break;
        case moveL_up:
            action4(msg_L, msg_R, deta_theta);
            if ((msg_L.Y_Axis - InitL_PosY) > ((theta_miu / 2) * deg2rad * 35))
                flag_ = turnL_anticlock;
            ROS_INFO("moveL_up");
            break;

        case turnL_anticlock:
            action2(msg_L, msg_R, -deta_theta);
            if (msg_L.Z_Angle < 0.0)
                flag_ = turnL_clock;
            ROS_INFO("turnL_anticlock");
            break;

        default:
            break;
        }

        ROS_INFO("LX: %lf", msg_L.X_Axis);
        ROS_INFO("LY: %lf", msg_L.Y_Axis);
        ROS_INFO("LZ_Angle: %lf", msg_L.Z_Angle);
        // ROS_INFO("RX: %lf",msg_R.X_Axis);
        // ROS_INFO("RY: %lf",msg_R.Y_Axis);
        // ROS_INFO("RZ_Angle: %lf",msg_R.Z_Angle);

        CmdR_pub.publish(msg_R);
        CmdL_pub.publish(msg_L);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}