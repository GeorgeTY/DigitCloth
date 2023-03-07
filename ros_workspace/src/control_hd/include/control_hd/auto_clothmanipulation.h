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
#include "control_hd/state.h"
#include "std_msgs/Int32.h"

// #define COUNT_MODE 10//总计运动十次，用于测量移动距离
#define no_force_control
#define Modify_digit_sensor
#define PI 3.14159265
#define Radius 35.0
#define deg2rad  (PI/180.0)
#define rad2deg  (180.0/PI)
#define InitL_PosX -32.0//-28.0
#define InitR_PosX  2// 0
#define InitL_PosY 135.0
#define InitR_PosY 135.0
#define InitL_PosAngle 0.0
#define InitR_PosAngle 0.0
#define Modify 0.065
#define frequency 15
#define Target_height 0.5//0.5
#define Theta_threshold 50.0
#define w_deg  5.0


#define Rstart_threahold -10.0
#define Rmid_threshold 0.0
#define Rend_threshold 20.0
#define Rend_angle 30.00
#define Max_dis 9 // 砂纸区域宽度，即理论单步最远移动距离

using namespace std;

class Move_cloth{
    private:
        enum flag{
            Init_position=0,
            Force_control, // 中央抓握，进行力控
            InitPos_AfterForce, // 根据目标移动距离，初始化弧形指尖位姿
            Turn_action, // 第一元动作，初始位置应该根据目标移动距离来计算
            Slid_action, 
            Roll_action, 
            Stop, // 停止过程
            Rest, // 停止状态，保持恢复中央抓握状态
            Release, // 释放夹爪
            Final
        };
        flag flag_;

        float deta_theta;
        const float w=w_deg;
        ros::Duration init_duration;
        ros::Time init_begin;
     
        const float target_height=Target_height;
        const float theta_len=10.0;
        const float theta_miu=(theta_len/Radius)*rad2deg;
        const float theta_threshold=Theta_threshold; //接触面为高摩擦区对应的弧度角
        float deta_ythreshold;
        float start_angle;


        /*力相关*/
        float sum_height;
        float upArea_height;
        float downArea_height;
        float init_sum_height; //静态噪声值
        const float KP= 0.2;
        const float KI= 0.0;
        const float KD= 0.2;
        float E1;
        float E2;
        float E3;

        double angle_turn; // 旋转的角度，用于计数累加，
        double Turn_startangel; // 用于存储目标移动距离计算出来的旋转角大小
        // /*以角度限制作为，角度阈值设置*/
        // float deta_turn;
        // float deta_slid;
        // float deta_roll;
        // float turn_threshold;
        // float slid_threshold;
        // float roll_threshold;

        ros::NodeHandle node;
        ros::Publisher detectDis_pub;
        std_msgs::Int32 detect_dis;
        bool flag_pub; // 用于标记当前的单元有没有发动过检测距离
        

    public:
        int count; //统计操作次数
        
    private:
        void actionTurn(hd_servo::EndPos &msg_L,hd_servo::EndPos &msg_R,float deta_theta);
        void actionSlid(hd_servo::EndPos &msg_L,hd_servo::EndPos &msg_R,float deta_theta);
        void actionRoll(hd_servo::EndPos &msg_L,hd_servo::EndPos &msg_R,float deta_theta);

        float getAnglefromDis(int nextUnit_dis);
    public:
        Move_cloth(){};
        ~Move_cloth(){};
        void Init(hd_servo::EndPos& msg_L,hd_servo::EndPos& msg_R);
        void setDistance(float distance);
        void MoveClothin(hd_servo::EndPos& msg_L,hd_servo::EndPos& msg_R,get_height::Height &current_height, int nextUnit_dis ,float optical_flow);
        string getHdControlstate();
        
};

#endif