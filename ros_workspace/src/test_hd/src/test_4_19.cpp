#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <numeric>
#include "hd_servo/EndPos.h"
#include "get_height/Height.h"
#include "cloth_manipulate.h"
#include "fingerTip_image/clothEdge.h"
#include "kukaservo_lib.h"

#define L_Pos_X_Axis -35
#define L_Pos_Y_Axis 140
#define L_Pos_Z_Angle 0
#define R_Pos_X_Axis 0
#define R_Pos_Y_Axis 140
#define R_Pos_Z_Angle 0


using namespace std;


int main(int argc,char **argv){

    ros::init(argc, argv, "test_hd");
    ros::NodeHandle nh;
    ros::Publisher CmdL_pub=nh.advertise<hd_servo::EndPos>("Goal_EndPos_L",1);
    ros::Publisher CmdR_pub=nh.advertise<hd_servo::EndPos>("Goal_EndPos_R",1);//当前策略只需要发布一次右侧手位置
    hd_servo::EndPos msg_L;
    hd_servo::EndPos msg_R;
    bool flag_cloth=false;
    msg_L.X_Axis=L_Pos_X_Axis;msg_L.Y_Axis=L_Pos_Y_Axis;msg_L.Z_Angle=L_Pos_Z_Angle;
    msg_R.X_Axis=R_Pos_X_Axis;msg_R.Y_Axis=R_Pos_Y_Axis;msg_R.Z_Angle=R_Pos_Z_Angle;


    /*先service call设置机械臂的运动模式*/

    bool flag_clock=true;
    ros::Rate loop_rate(frequency);  //频率高会导致只有单手能收到信号 不知道与什么有关
    while (ros::ok())
    {   
        if(flag_clock){
            msg_L.Z_Angle-=0.1;
            if(msg_L.Z_Angle<-20)
                flag_clock=false;
        }else{
            msg_L.Z_Angle+=0.1;
            if(msg_L.Z_Angle>20)
                flag_clock=true;
        }
        ROS_INFO("LX: %lf",msg_L.X_Axis);
        ROS_INFO("LY: %lf",msg_L.Y_Axis);
        ROS_INFO("LZ_Angle: %lf",msg_L.Z_Angle);
        ROS_INFO("RX: %lf",msg_R.X_Axis);
        ROS_INFO("RY: %lf",msg_R.Y_Axis);
        ROS_INFO("RZ_Angle: %lf",msg_R.Z_Angle);
        CmdR_pub.publish(msg_R);
        CmdL_pub.publish(msg_L);
            
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;   
}