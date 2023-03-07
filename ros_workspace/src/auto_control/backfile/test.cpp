#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <numeric>
#include "hd_servo/EndPos.h"
#include "get_height/Height.h"

using namespace std;
#define PI 3.14159265
#define deg2rad  (PI/360.0)



void Digit_callback(const get_height::Height::ConstPtr& msg,get_height::Height &current_height){
    ROS_INFO("Callback: get digit info !");
    current_height=*msg;  
    return;
}

vector<float> Calculate_SubHeight(const get_height::Height &current_height){
    vector<float> SubHeight(6,0.0);         //将传感器数据初步分为6个区数据,传感器数据为6*3
    SubHeight[0]=current_height.height_data0[0]+current_height.height_data0[1]+current_height.height_data0[2];
    SubHeight[1]=current_height.height_data1[0]+current_height.height_data1[1]+current_height.height_data1[2];
    SubHeight[2]=current_height.height_data2[0]+current_height.height_data2[1]+current_height.height_data2[2];
    SubHeight[3]=current_height.height_data3[0]+current_height.height_data3[1]+current_height.height_data3[2];
    SubHeight[4]=current_height.height_data4[0]+current_height.height_data4[1]+current_height.height_data4[2];
    SubHeight[5]=current_height.height_data5[0]+current_height.height_data5[1]+current_height.height_data5[2];
    // ROS_INFO("Sub0: %lf",SubHeight[0]);
    // ROS_INFO("Sub1: %lf",SubHeight[1]);
    // ROS_INFO("Sub2: %lf",SubHeight[2]);
    // ROS_INFO("Sub3: %lf",SubHeight[3]);
    // ROS_INFO("Sub4: %lf",SubHeight[4]);
    // ROS_INFO("Sub5: %lf",SubHeight[5]);

    return SubHeight;
}

int main(int argc,char **argv){


    ros::init(argc, argv, "test_hd");
    ros::NodeHandle nh;
    ros::Publisher CmdL_pub=nh.advertise<hd_servo::EndPos>("Goal_EndPos_L",1);
    ros::Publisher CmdR_pub=nh.advertise<hd_servo::EndPos>("Goal_EndPos_R",1);//当前策略只需要发布一次右侧手位置
    hd_servo::EndPos msg_L;
    hd_servo::EndPos msg_R;
    msg_L.X_Axis=0;msg_L.Y_Axis=140;msg_L.Z_Angle=0.0;
    msg_R.X_Axis=0;msg_R.Y_Axis=140;msg_R.Z_Angle=0.0;

    get_height::Height current_height;
    /*Ldigit_height现被安装在Finger_R*/
    ros::Subscriber DigitL_sub=nh.subscribe<get_height::Height>("/Ldigit_height",1,boost::bind(&Digit_callback,_1,ref(current_height)));
    float sum_height;
    float upArea_height;
    float downArea_height;
    const float target_height=3.0;



    const float Kp= 0.2;
    const float Ki= 0.0; 
    const float Kd= 0.25;

    const float KP= 0.2;
    const float KI= 0.0;
    const float KD= 0.2;

    float e1=0;
    float e2=0;
    float e3=0;

    float E1=0;
    float E2=0;
    float E3=0;


    int flag_dirORtrans=0;
    ros::Rate loop_rate(20);  //频率高会导致只有单手能收到信号 不知道与什么有关
    while (ros::ok())
    {
        vector<float> sub_height=Calculate_SubHeight(current_height);
        sum_height=accumulate(sub_height.begin(),sub_height.end(),0.0);
        upArea_height=accumulate(sub_height.begin(),sub_height.begin()+4,0.0);
        downArea_height=accumulate(sub_height.begin()+4,sub_height.end(),0.0);

        ROS_INFO("Sum_force: %lf",sum_height);
        ROS_INFO("Up_force: %lf",upArea_height);
        ROS_INFO("Down_force: %lf",downArea_height);
        
        if(flag_dirORtrans==0){
            e1=e2;
            e2=e3;
            e3=upArea_height-downArea_height;
            float deta_theta=Kp*e3+Ki*(e3+e2+e1)+Kd*((e3-e2)-(e2-e1));
            deta_theta=abs(deta_theta)>1.0?(deta_theta/abs(deta_theta)*1.0):(deta_theta);
            msg_R.Z_Angle=msg_R.Z_Angle-deta_theta;  //对于左手指，逆时针为负，顺时针为正；右手指则相反
            // ROS_INFO("deta_theta : %lf",deta_theta);
            // flag_dirORtrans=1;
        }else{
            E1=E2;
            E2=E3;
            E3=target_height-sum_height;
            float deta_height=KP*E3+KI*(E1+E2+E3)+KD*((E3-E2)-(E2-E1));
            deta_height=abs(deta_height)>1.0?(deta_height/abs(deta_height)*1.0):deta_height;
            msg_R.X_Axis=msg_R.X_Axis-deta_height*cos(msg_R.Z_Angle*deg2rad);
            msg_R.Y_Axis=msg_R.Y_Axis-deta_height*sin(msg_R.Z_Angle*deg2rad);
            flag_dirORtrans=0;
            // ROS_INFO("deta_height : %lf",deta_height);
             
        }
        ROS_INFO("X: %lf",msg_R.X_Axis);
        ROS_INFO("Y: %lf",msg_R.Y_Axis);
        ROS_INFO("Z_Angle: %lf",msg_R.Z_Angle);
        
        CmdR_pub.publish(msg_R);
        // CmdL_pub.publish(msg_L);
      
     
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
    
}