#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <numeric>
#include <string>

#include "hd_servo/EndPos.h"
// #include "digitcloth/cloth_manipulate.h"
#include "digitcloth/Edge_msg.h"
#include "digitcloth/move_cloth.h"

void edgeCallback(const digitcloth::Edge_msg::ConstPtr &msg)
{
    if (msg->isEdge == true)
    {
        ROS_INFO("Edge detected!");
    }
    else
    {
        ROS_INFO("No edge detected!");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "digitcloth");
    ros::NodeHandle nh;
    ros::Rate rosRate(10);

    ros::Publisher CmdL_pub = nh.advertise<hd_servo::EndPos>("Goal_EndPos_L", 1);
    ros::Publisher CmdR_pub = nh.advertise<hd_servo::EndPos>("Goal_EndPos_R", 1);
    hd_servo::EndPos msg_L;
    hd_servo::EndPos msg_R;

    ros::Subscriber Edge_sub = nh.subscribe<digitcloth::Edge_msg>("digitcloth_edge", 1000, edgeCallback);

    moveCloth myCloth;

    ros::Time tic = ros::Time::now();
    ros::Time toc = ros::Time::now();

    bool isInit = false;
    bool isEdgeDetected = false;

    ROS_INFO("Starting.");
    while (ros::ok())
    {
        if (!isInit)
        {
            isInit = true;
            myCloth.moveInit(msg_L, msg_R);
            myCloth.calcDistance(3);
            ROS_INFO("Ready.");
            ros::Duration(3).sleep(); // Wait for Preparation
            myCloth.moveGrab(msg_L, msg_R);
        }

        myCloth.moveClothInwards(msg_L, msg_R, 0.2);
        if (isEdgeDetected)
        {
            ROS_INFO("Edge Detected.");
            break;
        }

        CmdR_pub.publish(msg_R);
        CmdL_pub.publish(msg_L);

        ros::spinOnce();
        rosRate.sleep();
    }
    return 0;
}