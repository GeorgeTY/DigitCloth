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

        // /*返回Lefthome位置*/
        // void MoveLeftToHome(double time=10);
        // /*以相机坐标系为准，移动机械臂以指定位姿移动到指定点       alpha为沿X轴旋转到的目标角度    beta为沿Y轴旋转到的目标角度   gamma为沿Z轴旋转到的目标角度*/
        // void MoveLeftEulerXYZ(double X_Axis=0.6,double Y_Axis=0.225,double Z_Axis=0.669,double alpha=0,double beta=0,double gamma=-90,double time=10,int path=0);
        // /*以相机坐标系为准，移动机械臂以指定位姿移动到指定点       x,y,z,w为目标点四元数*/
        // void MoveLeftQuaternion(double X_Axis=0.6,double Y_Axis=0.225,double Z_Axis=0.669,double x=0,double y=0,double z=-0.707,double w=0.707,double time=10,int path=0);
        // /*指定机械臂七个关节角进行移动*/
        // void MoveLeftJoint(double Joint1=-23.09,double Joint2=-29.527,double Joint3=4.9144,double Joint4=-72.56458,double Joint5=25.49335,double Joint6=46.8971,double Joint7=9.2455,double time=10,int path=0);
        // /*机械臂相对于当前位置移动一个坐标和位姿,位姿用EulerXYZ表示*/
        // void MoveDLeftEulerXYZ(double dx=0,double dy=0,double dz=0,double dalpha=0,double dbeta=0,double dgamma=0,double time=10,int path=0);
        // /*机械臂相对于当前位置移动一个坐标和位姿，位姿用四元数表示*/
        // void MoveDLeftQuaternion(double dX=0,double dY=0,double dZ=0,double dx=0,double dy=0,double dz=0,double dw=0,double time=10,int path=0);
        // /*机械臂各关节角相对于当前关节角移动一个关节角,单位为°*/
        // void MoveDLeftJoint(double dJoint1=0,double dJoint2=0,double dJoint3=0,double dJoint4=0,double dJoint5=0,double dJoint6=0,double dJoint7=0,double time=10,int path=0);
        // /*设置运动模式*/
        // void SetLeftMoveMode(int moveMode=0,int pathMode=0,double moveDuration=10);
        

int main(int argc,char **argv){
    ros::init(argc, argv, "test_hd");
    ros::NodeHandle nh;

    float sum_Deta=0;
    Servo_Lib kuka_control;
    float deta_x=0;
    float deta_y=0;
    float deta_z=0;
    float deta_alpha=10.0;
    float deta_beta=0;
    float deta_gamma=0;

    float x=0.6;
    float y=0.22;
    float z=0.7;
    float alpha=-10;
    float beta=0;
    float gamma=-90;


    float time=5;
    int path_mode=0;

    int frequency_=1;
    // float time_=1/(float)frequency_;
    ros::Rate loop_rate(frequency_);  //频率高会导致只有单手能收到信号 不知道与什么有关
    
    ros::Time start_time=ros::Time::now();//循环内计时用，方便状态切换
    ros::Duration duration_threshold(time);


    while(ros::ok()){
        // kuka_control.MoveDLeftEulerXYZ(deta_x,deta_y,deta_y,deta_alpha,deta_beta,deta_gamma,time,path_mode);///////////
        kuka_control.MoveLeftEulerXYZ(x,y,z,alpha,beta,gamma,time,path_mode);

        // ros::Duration(time+0.5).sleep();
        if(ros::Time::now()-start_time>duration_threshold){
            start_time=ros::Time::now();
            // deta_alpha=-deta_alpha;
            alpha=-alpha;
            // break;
        }
        ROS_INFO("dd");
        ros::spinOnce();
        loop_rate.sleep();
    }

    
    return 0;   
}