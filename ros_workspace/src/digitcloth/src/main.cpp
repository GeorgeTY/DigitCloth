#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <numeric>
#include <string>

#include "hd_servo/EndPos.h"
#include "digitcloth/cloth_manipulate.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "digitcloth");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    ros::Publisher CmdL_pub = nh.advertise<hd_servo::EndPos>("Goal_EndPos_L", 1);
    ros::Publisher CmdR_pub = nh.advertise<hd_servo::EndPos>("Goal_EndPos_R", 1);
    hd_servo::EndPos msg_L;
    hd_servo::EndPos msg_R;

    Move_cloth MoveCloth;
    MoveCloth.Init(msg_L, msg_R);
    MoveCloth.Set_MoveDistance_in(8, msg_L);

    ros::Time tic = ros::Time::now();
    ros::Time toc = ros::Time::now();

    ROS_INFO("Starting.");
    while (ros::ok())
    {
        if (ros::Time::now() - tic < ros::Duration(3))
            continue;
        MoveCloth.Move_Cloth_in(msg_L, msg_R);

        CmdR_pub.publish(msg_R);
        CmdL_pub.publish(msg_L);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}