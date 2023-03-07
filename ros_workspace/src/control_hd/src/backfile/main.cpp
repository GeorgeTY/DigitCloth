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

using namespace std;

enum experiment_flag{
    Experiment_Init=0,
    InsertHd_ClothBottom,
    RollUp_Cloth,
    Make_ClothLayered,
    Grab_SingleCloth,
    TakeUp_SingleCloth, 
    Move_ClothTarget,
    Release_Cloth,
    MoveArmL_SafePos,
    Experiment_Finished
};
class Hd_Control{
public:
        /*---------------------------夹爪的初始化状态应该具体调整，方便夹爪能够嵌入布料下方-------------------------*/
    void Init_Finger(hd_servo::EndPos& msg_L,hd_servo::EndPos& msg_R){
        msg_L.X_Axis=-35-10;msg_L.Y_Axis=135;msg_L.Z_Angle=0;//x的值不变
        msg_R.X_Axis=0+10;msg_R.Y_Axis=135;msg_R.Z_Angle=0;
        return;
    }
    /*-------------------------------------------------------------------------------------------------*/

    void Adjust_Finger(hd_servo::EndPos& msg_L,hd_servo::EndPos& msg_R){
        /*设置夹爪为根部接触，以抓取单层*/
        msg_L.X_Axis=msg_L.X_Axis;msg_L.Y_Axis=InitL_PosY;msg_L.Z_Angle=-15.0;//x的值不变
        msg_R.X_Axis=msg_R.X_Axis;msg_R.Y_Axis=InitR_PosY;msg_R.Z_Angle=0;
        return;
    }
    void Release_Finger(hd_servo::EndPos& msg_L,hd_servo::EndPos& msg_R){
        /*释放夹爪，更改X方向位置，*/
        msg_L.X_Axis=msg_L.X_Axis-15;msg_L.Y_Axis=msg_L.Y_Axis;msg_L.Z_Angle=msg_L.Z_Angle;//x的值不变
        msg_R.X_Axis=msg_R.X_Axis+15;msg_R.Y_Axis=msg_R.Y_Axis;msg_R.Z_Angle=msg_R.Z_Angle;
        return;
    }
};

void Digit_callback(const get_height::Height::ConstPtr& msg,get_height::Height &current_height){
    ROS_INFO("Callback: get digit info !");
    current_height=*msg;  
    return;
}
void Edge_callback(const fingerTip_image::clothEdge::ConstPtr& msg,bool flag_){
    ROS_INFO("Callback: get fingerTip image");
    flag_=msg->edge_inView;
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
    ros::Subscriber Edge_detect=nh.subscribe<fingerTip_image::clothEdge>("cloth_Edge",1,boost::bind(&Edge_callback,_1,ref(flag_cloth)));
    
    get_height::Height current_height;
    /*Ldigit_height现被安装在Finger_R*/
    ros::Subscriber DigitL_sub=nh.subscribe<get_height::Height>("/Ldigit_height",1,boost::bind(&Digit_callback,_1,ref(current_height)));
    ros::Subscriber DigitR_sub=nh.subscribe<get_height::Height>("/Rdigit_height",1,boost::bind(&Digit_callback,_1,ref(current_height)));
    
    /*布料操作的夹爪控制类*/
    Move_cloth move_cloth_layered;
    experiment_flag Eflag_=Experiment_Init;

    /*机械臂的控制类*/
    Servo_Lib kuka_control;
    kuka_control.MoveDualToHome();

    Hd_Control hd_control;
    
    
    ros::Time start_time=ros::Time::now();//循环内计时用，方便状态切换
    ros::Duration duration_threshold;
    double deta_move=0.0; //用于循环内运动距离判断
    
    /*先service call设置机械臂的运动模式*/

    ros::Rate loop_rate(frequency);  //频率高会导致只有单手能收到信号 不知道与什么有关
    while (ros::ok())
    {   

        switch(Eflag_){
            case Experiment_Init:
                {
                    hd_control.Init_Finger(msg_L,msg_R);
                    // CmdR_pub.publish(msg_R);
                    // CmdL_pub.publish(msg_L);
                    /*首先进行织物的角点识别，根据织物位置信息，移动机械臂到目标附近位置，并初始化夹爪状态*/
                    kuka_control.MoveLeftEulerXYZ(L_Start_X_Axis,L_Start_Y_Axis,L_Start_Z_Axis,L_Start_Alpha,L_Start_Beta,L_Start_Gamma,0,10);
                    if(ros::Time::now()-start_time>ros::Duration(10)){
                        Eflag_=InsertHd_ClothBottom;
                        ROS_INFO("Start insert hand bottom of cloth");
                    }
                }
                break;
            case InsertHd_ClothBottom:
                {
                    /*把夹爪尖端插入布料下方*/
                    double deta_y=0.05; /***************具体移动方向以及大小以应试验后给出****************/
                    kuka_control.MoveDLeftEulerXYZ(0,deta_y,0,0,0,0,1);
                    deta_move+=deta_y;
                    if(deta_move>0.15/**********插入距离的阈值应该试验一下************/){
                        Eflag_=RollUp_Cloth;
                        deta_move=0;
                        ROS_INFO("Start roll up the cloth");
                    } 
                }
                break;

            case RollUp_Cloth:
                {
                    double deta_alpha=10;/***********************以某一轴为中心旋转，具体轴以及旋转角度应该经过试验*******************************/
                    kuka_control.MoveDLeftEulerXYZ(0,0,0,deta_alpha,0,0,1);
                    deta_move+=deta_alpha;
                    /**********使机械臂旋转，带动布料边角卷起，旋转的中心轴应该加上夹爪的偏置距离***********/
                    /**********所以需要更改kuka_kinematics.cpp文件中的FINGER_TIP_OFFSET*************/
                    if(deta_move>75.0/*状态切换条件为卷起角度*/){
                        Eflag_=Make_ClothLayered;
                        move_cloth_layered.Init(msg_L,msg_R);
                        move_cloth_layered.Set_MoveDistance_out(12,msg_L);
                        deta_move=0;
                        ROS_INFO("Start make cloth layered");
                    }
                }
                break;
            case Make_ClothLayered:
                {
                        
                    /*使布料分层*/
                    move_cloth_layered.Move_Cloth_out(msg_L,msg_R,current_height);
                    // CmdR_pub.publish(msg_R);
                    // CmdL_pub.publish(msg_L);
            
                    if(flag_cloth/*触发条件，指尖的传来的图像，其中黑色的像素，超过一定的阈值*/){
                        hd_control.Adjust_Finger(msg_L,msg_R);
                        start_time=ros::Time::now();
                        duration_threshold=ros::Duration(1);
                        Eflag_=Grab_SingleCloth;
                        ROS_INFO("Start grab single cloth");
                    }
                }
                break;
            case Grab_SingleCloth:
                {
                        /*调整抓取位置，使得指根抓取*/
                    // CmdR_pub.publish(msg_R);
                    // CmdL_pub.publish(msg_L);

                    if((ros::Time::now()-start_time)>duration_threshold){
                        Eflag_=TakeUp_SingleCloth;
                        ROS_INFO("Start take up the single cloth");
                        deta_move=0;
                    }
                }

                break;
            case TakeUp_SingleCloth:
                {
                    double deta_y=0.05;
                    double deta_z=0.05; //***************具体移动方向,移动方向应该是斜向的以及大小以应试验后给出****************
                    kuka_control.MoveDLeftEulerXYZ(0,deta_y,deta_z,0,0,0,1);
                    deta_move+=deta_z;
                    if(deta_move>0.1/*以向上移动的距离为条件*/){
                        Eflag_=Move_ClothTarget;
                        /*机械臂的target暂时通过自定义给出*/
                        start_time=ros::Time::now();
                        duration_threshold=ros::Duration(10);
                        deta_move=0;
                        ROS_INFO("Start move cloth towards target");
                    }
                }

                break;
            case Move_ClothTarget:
                {
                    kuka_control.MoveLeftEulerXYZ(L_Target_X_Axis,L_Target_Y_Axis,L_Target_Z_Axis,L_Target_Alpha,L_Target_Beta,L_Target_Gamma,0,10);
                    if(ros::Time::now()-start_time>duration_threshold){
                        Eflag_=Release_Cloth;
                        hd_control.Release_Finger(msg_L,msg_R);
                        start_time=ros::Time::now();
                        duration_threshold=ros::Duration(1);
                        ROS_INFO("Release the cloth");
                    }
                }
                break;

            case Release_Cloth:
                {
                    // CmdR_pub.publish(msg_R);
                    // CmdL_pub.publish(msg_L);
                    /*释放的同时，机械臂上抬*/
                    //递增发布增加Z指令
                    if(ros::Time::now()-start_time>duration_threshold){
                        Eflag_=MoveArmL_SafePos;
                        kuka_control.MoveDLeftEulerXYZ(0,0,0.1,0,0,0,1); //把机械臂上抬，注意是在什么坐标系下，也即注意增量的正负
                        start_time=ros::Time::now();
                        duration_threshold=ros::Duration(1);
                    }
                }
                break;
            case MoveArmL_SafePos:
                {
                    if(ros::Time::now()-start_time>duration_threshold){
                        Eflag_=Experiment_Finished;
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