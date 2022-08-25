#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <numeric>
#include <string>

#include "hd_servo/EndPos.h"
#include "digitcloth/cloth_manipulate.h"

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
// void Edge_callback(const fingerTip_image::clothEdge::ConstPtr &msg, bool &flag_)
// {
//     // ROS_INFO("Callback: get fingerTip image");
//     flag_ = msg->edge_inView;

//     return;
// }
int main(int argc, char **argv)
{
    ros::init(argc, argv, "digitcloth");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    ros::Publisher CmdL_pub = nh.advertise<hd_servo::EndPos>("Goal_EndPos_L", 1);
    ros::Publisher CmdR_pub = nh.advertise<hd_servo::EndPos>("Goal_EndPos_R", 1);
    hd_servo::EndPos msg_L;
    hd_servo::EndPos msg_R;

    Hd_Control hd_control;

    ros::Time tic = ros::Time::now();
    ros::Time toc = ros::Time::now();

    while (ros::ok())
    {
        /* Main Loop */
        switch (true)
        {
        case false:
            break;

        default:
            toc = ros::Time::now();
            if (tic - toc < ros::Duration(5))
            {
                hd_control.Init_Finger(msg_L, msg_R);
            }
            break;
        }

        CmdR_pub.publish(msg_R);
        CmdL_pub.publish(msg_L);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}