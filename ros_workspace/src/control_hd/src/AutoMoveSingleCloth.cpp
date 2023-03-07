#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <numeric>

#include "auto_clothmanipulation.h"
#include "hd_servo/EndPos.h"
#include "get_height/Height.h"
#include "control_hd/state.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"


using namespace std;
int flag_SIGINT = 1;

void Digit_callback(const get_height::Height::ConstPtr& msg,get_height::Height &current_height){
    // ROS_INFO("Callback: get digit info !");
    current_height=*msg;  
    return;
}

void Target_callback(const std_msgs::Int32::ConstPtr& msg, int &target_distance) {
    target_distance = msg->data;
    return;
}

void OpticalFlow_callback(const std_msgs::Float32::ConstPtr& msg, float &optical_flow) {
    optical_flow = msg->data;
    return;
}

int main(int argc,char **argv){

    ros::init(argc, argv, "control_hd");
    ros::NodeHandle nh;
    ros::Publisher CmdL_pub=nh.advertise<hd_servo::EndPos>("Goal_EndPos_L",1);
    ros::Publisher CmdR_pub=nh.advertise<hd_servo::EndPos>("Goal_EndPos_R",1);//当前策略只需要发布一次右侧手位置
    hd_servo::EndPos msg_L;
    hd_servo::EndPos msg_R;
    msg_L.X_Axis=InitL_PosX;msg_L.Y_Axis=132.0;msg_L.Z_Angle=InitL_PosAngle;
    msg_R.X_Axis=0;msg_R.Y_Axis=135.0;msg_R.Z_Angle=InitR_PosAngle;
    
    ros::Publisher state_pub=nh.advertise<control_hd::state>("Cloth_Maipulation_State",1);
    control_hd::state cloth_manipulation_state;

    get_height::Height current_height;
    // std_msgs::Int32 target_distance;
    int target_distance = 0;
    /*Ldigit_height现被安装在Finger_R*/
    ros::Subscriber DigitL_sub=nh.subscribe<get_height::Height>("/Ldigit_height",1,boost::bind(&Digit_callback,_1,ref(current_height)));
    ros::Subscriber DigitR_sub=nh.subscribe<get_height::Height>("/Rdigit_height",1,boost::bind(&Digit_callback,_1,ref(current_height)));
    
    // 订阅目标移动距离
    ros::Subscriber Distance_sub = nh.subscribe<std_msgs::Int32>("/Target_Distance",1,boost::bind(&Target_callback,_1,ref(target_distance)));
    // 订阅光流位移
    float optical_flow = 0;
    ros::Subscriber OpticalFlow_sub = nh.subscribe<std_msgs::Float32>("Turn_action_OpticalFlow", 1 , boost::bind(&OpticalFlow_callback,_1,ref(optical_flow)));

    Move_cloth move_cloth_in;
    move_cloth_in.Init(msg_L,msg_R);

    ros::Rate loop_rate(10);  //频率高会导致只有单手能收到信号 不知道与什么有关
     
    ros::Time start_time=ros::Time::now();
    while (ros::ok() && flag_SIGINT)
    {   
        if(ros::Time::now()-start_time<ros::Duration(10))
            continue;
        // 后期应该把Move_cloth_in的最后一个参数，目标移动距离作为话题消息
        move_cloth_in.MoveClothin(msg_L, msg_R, current_height, target_distance, optical_flow);

        /*布料操作状态，共有几种：
        1)Init_position
        2)Force_control
        3)Init_Pos_after_force
        4)Turn_action
        5)Slid_action
        6)Roll_action
        */
        cloth_manipulation_state.state = move_cloth_in.getHdControlstate();
        state_pub.publish(cloth_manipulation_state);

        // ROS_INFO("LX: %lf",msg_L.X_Axis);
        // ROS_INFO("LY: %lf",msg_L.Y_Axis);
        // ROS_INFO("LZ_Angle: %lf",msg_L.Z_Angle);
        // ROS_INFO("RX: %lf",msg_R.X_Axis);
        // ROS_INFO("RY: %lf",msg_R.Y_Axis);
        // ROS_INFO("RZ_Angle: %lf",msg_R.Z_Angle);

        // ROS_INFO("first_row_height%lf %lf %lf",current_height.height_data0[0],current_height.height_data0[1],current_height.height_data0[2]);
        // ROS_INFO("second_row_height%lf %lf %lf",current_height.height_data1[0],current_height.height_data1[1],current_height.height_data1[2]);
        // ROS_INFO("third_row_height%lf %lf %lf",current_height.height_data2[0],current_height.height_data2[1],current_height.height_data2[2]);
        // ROS_INFO("forth_row_height%lf %lf %lf",current_height.height_data3[0],current_height.height_data3[1],current_height.height_data3[2]);
        // ROS_INFO("fifth_row_height%lf %lf %lf",current_height.height_data4[0],current_height.height_data4[1],current_height.height_data4[2]);
        // ROS_INFO("sixth_row_height%lf %lf %lf",current_height.height_data5[0],current_height.height_data5[1],current_height.height_data5[2]);
        
        CmdR_pub.publish(msg_R);
        CmdL_pub.publish(msg_L);
      
     
        ros::spinOnce();
        loop_rate.sleep();
    }

    // ROS_INFO("Sum Move Step: %d",move_cloth_in.count);
    return 0;
}