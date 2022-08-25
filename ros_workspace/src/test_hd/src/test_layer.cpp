#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <numeric>
#include <string>
#include "hd_servo/EndPos.h"
#include "get_height/Height.h"
#include "test_hd/cloth_manipulate.h"
#include "fingerTip_image/clothEdge.h"
#include "test_hd/kukaservo_lib.h"

#define L_Start_X_Axis 0
#define L_Start_Y_Axis 0
#define L_Start_Z_Axis 0
#define L_Start_Alpha 0
#define L_Start_Beta 0
#define L_Start_Gamma 0

#define L_Target_X_Axis 0
#define L_Target_Y_Axis 0
#define L_Target_Z_Axis 0
#define L_Target_Alpha 0
#define L_Target_Beta 0
#define L_Target_Gamma 0
#define test

#define X_POS (0.2 + 0.1)
#define Target_X (-0.25 + 0.1)
using namespace std;

enum experiment_flag
{
    Experiment_Init = 0,
    InsertHd_ClothBottom,
    GrabTight_Cloth,
    RollUp_Cloth,
    Make_ClothLayered,
    Grab_SingleCloth,
    TakeUp_SingleCloth,
    Move_ClothTarget,
    Release_Cloth,
    MoveArmL_SafePos,
    Experiment_Finished
};
class Hd_Control
{
public:
    /*---------------------------夹爪的初始化状态应该具体调整，方便夹爪能够嵌入布料下方-------------------------*/
    void Init_Finger(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R)
    {
        msg_L.X_Axis = -35 - 10;
        msg_L.Y_Axis = 135;
        msg_L.Z_Angle = 0; // x的值不变
        msg_R.X_Axis = 0 + 15;
        msg_R.Y_Axis = 135;
        msg_R.Z_Angle = 0;
        return;
    }

    void grab_Finger(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R)
    {
        msg_L.X_Axis = -35;
        msg_L.Y_Axis = 135;
        msg_L.Z_Angle = 0; // x的值不变
        msg_R.X_Axis = 0;
        msg_R.Y_Axis = 135;
        msg_R.Z_Angle = 0;
        return;
    }
    /*-------------------------------------------------------------------------------------------------*/

    void Adjust_Finger(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R)
    {
        /*设置夹爪为根部接触，以抓取单层*/
        msg_L.X_Axis = msg_L.X_Axis;
        msg_L.Y_Axis = InitL_PosY;
        msg_L.Z_Angle = -15.0; // x的值不变
        msg_R.X_Axis = msg_R.X_Axis;
        msg_R.Y_Axis = InitR_PosY;
        msg_R.Z_Angle = 0;
        return;
    }
    void Release_Finger(hd_servo::EndPos &msg_L, hd_servo::EndPos &msg_R)
    {
        /*释放夹爪，更改X方向位置，*/
        msg_L.X_Axis = msg_L.X_Axis - 15;
        msg_L.Y_Axis = msg_L.Y_Axis;
        msg_L.Z_Angle = msg_L.Z_Angle; // x的值不变
        msg_R.X_Axis = msg_R.X_Axis + 15;
        msg_R.Y_Axis = msg_R.Y_Axis;
        msg_R.Z_Angle = msg_R.Z_Angle;
        return;
    }
};

void Digit_callback(const get_height::Height::ConstPtr &msg, get_height::Height &current_height)
{
    // ROS_INFO("Callback: get digit info !");
    current_height = *msg;
    return;
}
void Edge_callback(const fingerTip_image::clothEdge::ConstPtr &msg, bool &flag_)
{
    // ROS_INFO("Callback: get fingerTip image");
    flag_ = msg->edge_inView;

    return;
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_hd");
    ros::NodeHandle nh;
    ros::Publisher CmdL_pub = nh.advertise<hd_servo::EndPos>("Goal_EndPos_L", 1);
    ros::Publisher CmdR_pub = nh.advertise<hd_servo::EndPos>("Goal_EndPos_R", 1); //当前策略只需要发布一次右侧手位置
    hd_servo::EndPos msg_L;
    hd_servo::EndPos msg_R;
    bool flag_cloth = false;
    ros::Subscriber Edge_detect = nh.subscribe<fingerTip_image::clothEdge>("cloth_Edge", 1, boost::bind(&Edge_callback, _1, ref(flag_cloth)));

    get_height::Height current_height;
    /*Ldigit_height现被安装在Finger_R*/
    ros::Subscriber DigitL_sub = nh.subscribe<get_height::Height>("/Ldigit_height", 1, boost::bind(&Digit_callback, _1, ref(current_height)));
    ros::Subscriber DigitR_sub = nh.subscribe<get_height::Height>("/Rdigit_height", 1, boost::bind(&Digit_callback, _1, ref(current_height)));

    /*布料操作的夹爪控制类*/
    Move_cloth move_cloth_layered;
    string pre_HdState = move_cloth_layered.get_HdControl_state();

    experiment_flag Eflag_ = Experiment_Init;

    /*机械臂的控制类*/
    Servo_Lib kuka_control;
    kuka_control.MoveLeftToHome(10.0);

    Hd_Control hd_control;

    ros::Time start_time = ros::Time::now(); //循环内计时用，方便状态切换

    /*先service call设置机械臂的运动模式*/

    ros::Rate loop_rate(frequency); //频率高会导致只有单手能收到信号 不知道与什么有关
    while (ros::ok())
    {

        switch (Eflag_)
        {
        case Experiment_Init:
        {
            /*首先进行织物的角点识别，根据织物位置信息，移动机械臂到目标附近位置，并初始化夹爪状态*/
            if (ros::Time::now() - start_time < ros::Duration(5))
            {
                hd_control.Init_Finger(msg_L, msg_R);
            }
            else if (ros::Time::now() - start_time < ros::Duration(15))
            {
                kuka_control.MoveLeftEulerXYZ(X_POS, 0.15, 0.8, 0, -70, -180, 10, 0);
            }
            else if (ros::Time::now() - start_time < ros::Duration(20))
            {
                kuka_control.MoveLeftEulerXYZ(X_POS, 0.15, 0.998, 0, -70, -180, 5, 0);
            }
            else
            {
                Eflag_ = InsertHd_ClothBottom;
                ROS_INFO("Start insert hand bottom of cloth");
                start_time = ros::Time::now();
            }
        }
        break;
        case InsertHd_ClothBottom:
        {
            /*把夹爪尖端插入布料下方*/
            if (ros::Time::now() - start_time < ros::Duration(5))
            {
                kuka_control.MoveLeftEulerXYZ(X_POS - 0.1, 0.15, 0.998, 0, -70, -180, 5, 0);
            }
            else
            {
                Eflag_ = GrabTight_Cloth;
                ROS_INFO("Start grab tightly cloth");
                start_time = ros::Time::now();
                hd_control.grab_Finger(msg_L, msg_R);
            }
        }
        break;

        case GrabTight_Cloth:
        {
            ///*
            // move_cloth_layered.Move_Cloth_out(msg_L,msg_R,current_height);
            // string cur_HdState=move_cloth_layered.get_HdControl_state();
            // if(cur_HdState=="init_Pos_after_force"){
            //     Eflag_=RollUp_Cloth;
            //     ROS_INFO("Start roll up cloth");
            //     start_time=ros::Time::now();

            // }
            //*/
            if (ros::Time::now() - start_time > ros::Duration(2))
            {
                Eflag_ = RollUp_Cloth;
                ROS_INFO("Start roll up cloth");
                start_time = ros::Time::now();
            }
        }
        break;

        case RollUp_Cloth:
        {
            if (ros::Time::now() - start_time < ros::Duration(5))
            {
                kuka_control.MoveLeftEulerXYZ(X_POS - 0.13, 0.15, 0.95, 0, -70, -180, 5, 0);
            }
            else if (ros::Time::now() - start_time < ros::Duration(10))
            {
                kuka_control.MoveLeftEulerXYZ(X_POS - 0.13, 0.15, 0.95, 0, 0, -180, 5, 0);
            }
            else
            {
                Eflag_ = Make_ClothLayered;
                move_cloth_layered.Init(msg_L, msg_R);
                move_cloth_layered.Set_MoveDistance_out(12, msg_L);
                ROS_INFO("Start make cloth layered");
            }
        }
        break;
        case Make_ClothLayered:
        {

            /*使布料分层*/
            move_cloth_layered.Move_Cloth_out(msg_L, msg_R, current_height);
            string cur_HdState = move_cloth_layered.get_HdControl_state(); //根据状态信息的切换判断当前夹爪位置，保证只在布料紧贴亚克力表面时才进行分成检测

            if (pre_HdState == "turnL_clock" && cur_HdState == "turnL_anticlock" && flag_cloth /*触发条件，指尖的传来的图像，其中黑色的像素，超过一定的阈值*/)
            {
                hd_control.Adjust_Finger(msg_L, msg_R);
                start_time = ros::Time::now();
                Eflag_ = Grab_SingleCloth;
                ROS_INFO("Start grab single cloth");
            }
            pre_HdState = cur_HdState;
        }
        break;
        case Grab_SingleCloth:
        {
            /*调整抓取位置，使得指根抓取*/
            hd_control.Adjust_Finger(msg_L, msg_R);

            if ((ros::Time::now() - start_time) > ros::Duration(1))
            {
                Eflag_ = TakeUp_SingleCloth;
                ROS_INFO("Start take up the single cloth");
                start_time = ros::Time::now();
            }
        }

        break;
        case TakeUp_SingleCloth:
        {
            if (ros::Time::now() - start_time < ros::Duration(5))
            {
                kuka_control.MoveLeftEulerXYZ(X_POS - 0.13, 0.15, 0.90, 0, 0, -180, 5, 0);
            }
            else
            {
                Eflag_ = Move_ClothTarget;
                start_time = ros::Time::now();
                ROS_INFO("Start move cloth towards target");
            }
        }

        break;
        case Move_ClothTarget:
        {

            if (ros::Time::now() - start_time < ros::Duration(10))
            {
                kuka_control.MoveLeftEulerXYZ(Target_X, 0.15, 0.90, 0, 0, -180, 5, 0);
            }
            else
            {
                Eflag_ = Release_Cloth;
                hd_control.Release_Finger(msg_L, msg_R);
                start_time = ros::Time::now();
                ROS_INFO("Start move cloth towards target");
            }
        }
        break;

        case Release_Cloth:
        {
            /*等待释放*/
            if (ros::Time::now() - start_time > ros::Duration(1))
            {
                Eflag_ = MoveArmL_SafePos;
                start_time = ros::Time::now();
                ROS_INFO("Release Cloth");
            }
        }
        break;
        case MoveArmL_SafePos:
        {
            /*机械臂上抬*/
            if (ros::Time::now() - start_time < ros::Duration(5))
            {
                kuka_control.MoveLeftEulerXYZ(Target_X, 0.15, 0.8, 0, 0, -180, 5, 0);
            }
            else if (ros::Time::now() - start_time < ros::Duration(10))
            {
                kuka_control.MoveLeftEulerXYZ(X_POS, 0.15, 0.8, 0, 0.0, -180, 5, 0);
            }
            else
            {
                Eflag_ = Experiment_Finished;
                kuka_control.MoveLeftToHome();
            }
        }

        break;
        case Experiment_Finished:
            ROS_INFO("Experiment has been finished");
            break;

        default:
            break;
        }

        ROS_INFO("LX: %lf", msg_L.X_Axis);
        ROS_INFO("LY: %lf", msg_L.Y_Axis);
        ROS_INFO("LZ_Angle: %lf", msg_L.Z_Angle);
        ROS_INFO("RX: %lf", msg_R.X_Axis);
        ROS_INFO("RY: %lf", msg_R.Y_Axis);
        ROS_INFO("RZ_Angle: %lf", msg_R.Z_Angle);

        CmdR_pub.publish(msg_R);
        CmdL_pub.publish(msg_L);

        ROS_INFO("first_row_height%lf%lf%lf", current_height.height_data0[0], current_height.height_data0[1], current_height.height_data0[2]);
        ROS_INFO("second_row_height%lf%lf%lf", current_height.height_data1[0], current_height.height_data1[1], current_height.height_data1[2]);
        ROS_INFO("third_row_height%lf%lf%lf", current_height.height_data2[0], current_height.height_data2[1], current_height.height_data2[2]);
        ROS_INFO("forth_row_height%lf%lf%lf", current_height.height_data3[0], current_height.height_data3[1], current_height.height_data3[2]);
        ROS_INFO("fifth_row_height%lf%lf%lf", current_height.height_data4[0], current_height.height_data4[1], current_height.height_data4[2]);
        ROS_INFO("sixth_row_height%lf%lf%lf", current_height.height_data5[0], current_height.height_data5[1], current_height.height_data5[2]);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}