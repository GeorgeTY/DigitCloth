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

void edgeCallback(const digitcloth::Edge_msg::ConstPtr &msg, moveCloth &myCloth)
{
    if (msg->isEdge == true)
    {
        ROS_INFO("Edge detected!");
        myCloth.edge_detected.isEdgeDetected = true;
        myCloth.edge_detected.P0 = msg->p0;
        myCloth.edge_detected.P1 = msg->p1;
        myCloth.edge_detected.P2 = msg->p2;
    }
    else
    {
        // ROS_INFO("No edge detected!");
        myCloth.edge_detected.isEdgeDetected = false;
        myCloth.edge_detected.P0 = 0;
        myCloth.edge_detected.P1 = 0;
        myCloth.edge_detected.P2 = 0;
    }

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "digitcloth");
    ros::NodeHandle nh;
    ros::Rate rosRate(15);

    ros::Publisher CmdL_pub = nh.advertise<hd_servo::EndPos>("Goal_EndPos_L", 1);
    ros::Publisher CmdR_pub = nh.advertise<hd_servo::EndPos>("Goal_EndPos_R", 1);
    hd_servo::EndPos msg_L;
    hd_servo::EndPos msg_R;

    moveCloth myCloth;

    ros::Subscriber Edge_sub = nh.subscribe<digitcloth::Edge_msg>("digitcloth_edge", 1000, boost::bind(&edgeCallback, _1, ref(myCloth)));

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
            myCloth.calcDistance(10);
            ros::Duration(1).sleep(); // Wait for Preparation
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