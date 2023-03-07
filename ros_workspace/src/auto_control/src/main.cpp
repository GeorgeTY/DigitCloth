#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <math.h>
#include <numeric>
#include "control_hd/state.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

using namespace std;
#define Distance 40

void getActionState_callback(const control_hd::state::ConstPtr& msg,string &action_state){
    action_state = msg->state;
}
void getOpticalFlowDis_callback(const std_msgs::Float32::ConstPtr& msg,float& optical_dis) {
    optical_dis = msg->data;
}
void getDectedDis_callback(const std_msgs::Int32::ConstPtr& msg,int &last_dis, bool& flag) {
    last_dis = msg->data;
    flag = true;
}
int main(int argc,char **argv){
    // 订阅state与光流位移大小，检测单步移动距离，并进行规划，实现输入目标移动距离，能够发布单步运动目标，然后控制cotrol_hd进行调节

    ros::init(argc, argv, "auto_hd");
    ros::NodeHandle nh;
    string action_state;
    float optical_dis;
    int target_dis = Distance; // 预期移动距离
    ros::Subscriber actionstate_sub = nh.subscribe<control_hd::state>("Cloth_Maipulation_State",1,boost::bind(&getActionState_callback,_1,ref(action_state)));
   
    bool flag_recieve_detected_dis = true; // 收到检测到的移动距离 
    int last_dis = 0;
    ros::Subscriber opticalFlowTurn_sub = nh.subscribe<std_msgs::Int32>
            ("Cloth_Detected_Distance",1, boost::bind(&getDectedDis_callback,_1,ref(last_dis), ref(flag_recieve_detected_dis)));

    std_msgs::Int32 target_distance;
    ros::Publisher targetdis_pub=nh.advertise<std_msgs::Int32>("/Target_Distance",1);

    // ros::Time start_time=ros::Time::now();//循环内计时用，方便状态切换
    // ros::Duration duration_threshold;
    
    // target_distance.data = target_dis;
    // targetdis_pub.publish(target_distance);
    
    flag_recieve_detected_dis = true;
    ros::Rate loop_rate(1); 

    int count = 0;
    target_distance.data = 0;
    targetdis_pub.publish(target_distance);
    ros::Duration(2).sleep(); //确保targetdis_pub注册成功
    while (ros::ok()){

        if (count > 3) {
            target_distance.data = 0;
            targetdis_pub.publish(target_distance);
            ROS_INFO("Can not converge: error:%d",target_dis);
            break;
        }
        // ROS_INFO("OpticalFlow_Distance: %lf",optical_dis);
        // ROS_INFO("Action_state: %s",&action_state[0]);
        // 三种状态： 1)actionTurn; 2)actionSlid; 3)actionRoll;    
        
        if (target_dis < 1) {
            target_distance.data = 0;
            targetdis_pub.publish(target_distance);
            ROS_INFO("-----------------------------Finished-------------------------------");
            break;
        }

        if (flag_recieve_detected_dis) {
            // 为什么想让其只根据flag情况发送消息却发不出去呢
            ROS_INFO("last_detected_distance:%d",last_dis);
            target_dis -= last_dis;
            target_dis = target_dis > 0 ? target_dis : 0;
            target_distance.data = target_dis;
            targetdis_pub.publish(target_distance);
            flag_recieve_detected_dis = false; 

            if (last_dis == 0) {
                count ++;
            } else {
                count = 0;
            }
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;   
}