#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <numeric>
#include <string>
#include "hd_servo/EndPos.h"
#include "get_height/Height.h"
#include "cloth_manipulate.h"
#include "fingerTip_image/clothEdge.h"
#include "kukaservo_lib.h"

using namespace std;

enum experiment_flag{
    Init_robot=0,
    Push_Oncloth,
    Init_finger,
    Close_finger,
    Make_layered,
    Move_TouchPoint,
    Take_Upercloth,
    Move_ToTarget,
    Realse_Cloth,
    Finished
};
class Hd_Control{
public:
        /*---------------------------夹爪的初始化状态应该具体调整，方便夹爪能够嵌入布料下方-------------------------*/
    void Init_Finger(hd_servo::EndPos& msg_L,hd_servo::EndPos& msg_R){
        // msg_L.X_Axis=-50;msg_L.Y_Axis=130;msg_L.Z_Angle=-20;//x的值不变
        // msg_R.X_Axis=15;msg_R.Y_Axis=135;msg_R.Z_Angle=0;
        msg_L.X_Axis=-65;msg_L.Y_Axis=130;msg_L.Z_Angle=-20;//x的值不变
        msg_R.X_Axis=0;msg_R.Y_Axis=135;msg_R.Z_Angle=0;
        return;
    }
    void Close_Finger(hd_servo::EndPos& msg_L,hd_servo::EndPos& msg_R){
        msg_L.X_Axis=-30;msg_L.Y_Axis=130;msg_L.Z_Angle=-20;//x的值不变
        msg_R.X_Axis=0;msg_R.Y_Axis=135;msg_R.Z_Angle=0;
        return;
    }
    void Turn_Finger(hd_servo::EndPos& msg_L,hd_servo::EndPos& msg_R){
        msg_L.X_Axis=msg_L.X_Axis;msg_L.Y_Axis=msg_L.Y_Axis;msg_L.Z_Angle+=1.0;//x的值不变
        msg_R.X_Axis=msg_R.X_Axis;msg_R.Y_Axis=msg_R.Y_Axis;msg_R.Z_Angle=msg_R.Z_Angle;
        return;
    }

    void Move_TouchPoint(hd_servo::EndPos &msg_L,hd_servo::EndPos &msg_R,float deta_theta){
        /*V0_L=wR!=0,V_R=0;接触点相对Digit上移，接触点处两表面相对静止*/
        float modify_deta_y=deta_theta>0?Modify:-Modify;
        float deta_y=deta_theta*deg2rad*Radius+modify_deta_y;    //可能因为位置精度的问题，需要增加一个修正量，来保证速度的一致
        msg_L.Z_Angle=msg_L.Z_Angle+deta_theta;
        msg_L.Y_Axis=msg_L.Y_Axis+deta_y;
        return;
    }
    void Release_Cloth(hd_servo::EndPos &msg_L,hd_servo::EndPos &msg_R){
        msg_L.X_Axis=-65;msg_L.Y_Axis=130;msg_L.Z_Angle=-20;//x的值不变
        msg_R.X_Axis=0;msg_R.Y_Axis=135;msg_R.Z_Angle=0;
        return;
    }
};

void ClothDetect_callback(const fingerTip_image::clothEdge::ConstPtr& msg,bool &flag_){
    flag_=msg->edge_inView;
    // if(flag_)
    //     ROS_INFO("Callback: find cloth ");
    return ;
}

int main(int argc,char **argv){

    ros::init(argc, argv, "test_hd");
    ros::NodeHandle nh;
    ros::Publisher CmdL_pub=nh.advertise<hd_servo::EndPos>("Goal_EndPos_L",1);
    ros::Publisher CmdR_pub=nh.advertise<hd_servo::EndPos>("Goal_EndPos_R",1);//当前策略只需要发布一次右侧手位置
    hd_servo::EndPos msg_L;
    hd_servo::EndPos msg_R;
    
    bool flag_cloth=false;
    ros::Subscriber Cloth_detect=nh.subscribe<fingerTip_image::clothEdge>("Detect_cloth",1,boost::bind(&ClothDetect_callback,_1,ref(flag_cloth)));
   
    experiment_flag Eflag_=Init_robot; 
    ROS_INFO("Start init robot");


    Hd_Control hd_control;
    Servo_Lib kuka_control;

    ros::Time start_time=ros::Time::now();//循环内计时用，方便状态切换
    float deta_angle=0;
    /*先service call设置机械臂的运动模式*/

    ros::Rate loop_rate(15);  //频率高会导致只有单手能收到信号 不知道与什么有关
    while (ros::ok())
    {   

        switch(Eflag_){
            case Init_robot:
                {
                   kuka_control.MoveLeftEulerXYZ(0.2,0.15,0.8,0,0,-180,5,0); 
                   if(ros::Time::now()-start_time>ros::Duration(5)){
                        Eflag_=Init_finger;
                        start_time=ros::Time::now();
                        ROS_INFO("Start init finger");
                    }
                }
                break;
            case Init_finger:
                {
                    hd_control.Init_Finger(msg_L,msg_R);
                    if(ros::Time::now()-start_time>ros::Duration(5)){
                        Eflag_=Push_Oncloth;
                        start_time=ros::Time::now();
                        ROS_INFO("Start push on cloth");
                    }
                }
                break;

            case Push_Oncloth:
                {
                    kuka_control.MoveLeftEulerXYZ(0.2,0.15,1.009,0,0,-180,5,0); 
                    if(ros::Time::now()-start_time>ros::Duration(5)){
                        Eflag_=Close_finger;
                        start_time=ros::Time::now();
                        ROS_INFO("Start close finger");
                    }
                }
                break;

            case Close_finger:
                {
                    hd_control.Close_Finger(msg_L,msg_R);
                    if(ros::Time::now()-start_time>ros::Duration(5)&&flag_cloth){
                        Eflag_=Make_layered;
                        start_time=ros::Time::now();
                        ROS_INFO("Start make layered");
                    }else if(ros::Time::now()-start_time>ros::Duration(5)&&(!flag_cloth)){
                        start_time=ros::Time::now();
                        Eflag_=Init_finger;
                        ROS_INFO("Cloth not grab,Retry!");
                    }
                }
                break;
            
            case Make_layered:
                {
                    hd_control.Turn_Finger(msg_L,msg_R);
                    deta_angle+=1.0;
                    if(deta_angle>30.0){
                        Eflag_=Move_TouchPoint;
                        ROS_INFO("Start Move_TouchPoint");
                        deta_angle=0.0;
                        start_time=ros::Time::now();              
                    }     
                }
                break;
            case Move_TouchPoint:
                {
                    hd_control.Move_TouchPoint(msg_L,msg_R,-1.0);
                    deta_angle+=1.0;
                    if(deta_angle>14.0){
                        Eflag_=Take_Upercloth;
                        ROS_INFO("Start Take_Upercloth");
                        start_time=ros::Time::now();
                    }          
                }  
                break;  

            case Take_Upercloth:
                {
                    kuka_control.MoveLeftEulerXYZ(0.15,0.15,0.92,0,0,-180,5,0); 
                    if(ros::Time::now()-start_time>ros::Duration(5)){
                        Eflag_=Move_ToTarget;
                        start_time=ros::Time::now();
                        ROS_INFO("Start Move to target");
                    }
                }
                break;
            
            case Move_ToTarget:
                {
                    kuka_control.MoveLeftEulerXYZ(-0.10,0.15,0.92,0,0,-180,5,0); 
                    if(ros::Time::now()-start_time>ros::Duration(5)){
                        Eflag_=Realse_Cloth;
                        start_time=ros::Time::now();
                        ROS_INFO("Start Release the cloth");
                    }
                } 
                break;

            case Realse_Cloth:
                {
                    hd_control.Release_Cloth(msg_L,msg_R);
                    if(ros::Time::now()-start_time>ros::Duration(5)){
                        Eflag_=Finished;
                        start_time=ros::Time::now();
                        ROS_INFO("Finished");
                    }
                }
                break;

            case Finished:
                {
                    if(ros::Time::now()-start_time<ros::Duration(5))
                        kuka_control.MoveLeftEulerXYZ(-0.10,0.15,0.80,0,0,-180,5,0);
                    ROS_INFO("Finished");
                }
                break;
            default:
                break;

        }

        // ROS_INFO("LX: %lf",msg_L.X_Axis);
        // ROS_INFO("LY: %lf",msg_L.Y_Axis);
        // ROS_INFO("LZ_Angle: %lf",msg_L.Z_Angle);
        // ROS_INFO("RX: %lf",msg_R.X_Axis);
        // ROS_INFO("RY: %lf",msg_R.Y_Axis);
        // ROS_INFO("RZ_Angle: %lf",msg_R.Z_Angle);
        CmdR_pub.publish(msg_R);
        CmdL_pub.publish(msg_L);
            
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;   
}