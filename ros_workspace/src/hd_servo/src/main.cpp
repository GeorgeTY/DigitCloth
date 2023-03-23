#include <ros/ros.h>
#include <math.h>
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "hd_drive.h"
#include "system_time.h"
#include "calc_pathInterpolation.h"
#include "hd_kinematics.h"
#include "hd_servo/EndPos.h"
#include "hd_servo/JointSpeed.h"

using namespace std;

#define Duration_ 0.5 // 轨迹规划的总时间2.0s
#define Enable_Finger_L
#define Enable_Finger_R

/*订阅手指末端位姿的回调函数，接收到位姿指令后，重置手指类的目标位置，起始位置以及插值起始时间*/
void goalCallback(const hd_servo::EndPos::ConstPtr &goal_pos_, hd_drive &finger)
{
    /*重置目标位姿*/
    MATRIX_D goal_pos = MatD31(goal_pos_->X_Axis, goal_pos_->Y_Axis, goal_pos_->Z_Angle);
    finger.Goal_Pos = goal_pos;

    /*获取舵机状态，求解当前位姿态，作为新的路径规划起始位姿*/
    ros::Time start_time = ros::Time::now();

    finger.Motor_feedback();

    ros::Duration motorfeedback_time = ros::Time::now() - start_time;
    float feedbacktime = motorfeedback_time.toSec();
    ROS_INFO("MOTOR_FeedBack time:%lf", feedbacktime);

    finger.Origin_Pos = Forward_kinematics(finger.Joint_theta, finger.flag_);

    /*两个手的插值时间应该是不一样的，不可以共享一个全局时间变量，把时间变量封装在手的驱动类中，每个手指拥有自己的驱动时间*/
    SetCurtimeasStartTime(finger);

    if (finger.flag_ == 0)
        ROS_INFO("----------Finger_L received new goal position----------");
    else
        ROS_INFO("----------Finger_R received new goal position----------");

    /*for test*/
    ROS_INFO("-----------------Time Set ZERO -------------------");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_motor2");
    ros::NodeHandle n;

#ifdef Enable_Finger_L
    hd_drive finger_L_(Finger_L);
    finger_L_.Finger_init(); // 舵机中位校准及使能扭矩
    ros::Subscriber Goal_EndPosL_sub = n.subscribe<hd_servo::EndPos>("Goal_EndPos_L", 1, boost::bind(&goalCallback, _1, ref(finger_L_)));
#endif
#ifdef Enable_Finger_R
    hd_drive finger_R_(Finger_R);
    finger_R_.Finger_init();
    ros::Subscriber Goal_EndPosR_sub = n.subscribe<hd_servo::EndPos>("Goal_EndPos_R", 1, boost::bind(&goalCallback, _1, ref(finger_R_))); // bind函数传入变量的引用需要加ref
#endif

    // /*for test*/
    // ros::Publisher Cmd_EndPos_pub = n.advertise<hd_servo::EndPos>("EndPos_cmd", 10);//发布末端位姿命令
    // ros::Publisher Real_EndPos_pub = n.advertise<hd_servo::EndPos>("EndPos_real", 10);//反馈末端实际位置
    ros::Publisher MotorsR_Speed_pub = n.advertise<hd_servo::JointSpeed>("JointR_speed_feedback", 10); // 反馈舵机速度
    ros::Publisher MotorsL_Speed_pub = n.advertise<hd_servo::JointSpeed>("JointL_speed_feedback", 10); // 反馈舵机速度
    // FILE *InfoL_EndPos_Joint;//保存末端姿态以及关节指令角
    // InfoL_EndPos_Joint=fopen("/home/jhz/catkin_ws/src/hd_servo/src/test_output/TEST_L.txt","w+");
    // fprintf(InfoL_EndPos_Joint,"EndPos_X_Axis\tEndPos_Y_Axis\tEndPos_Angle\tJoint1_Cmd\tJoint2_Cmdt\tJoint3_Cmd\tTime\n");
    // FILE *InfoR_EndPos_Joint;
    // InfoR_EndPos_Joint=fopen("/home/jhz/catkin_ws/src/hd_servo/src/test_output/TEST_R.txt","w+");
    // fprintf(InfoR_EndPos_Joint,"EndPos_X_Axis\tEndPos_Y_Axis\tEndPos_Angle\tJoint1_Cmd\tJoint2_Cmdt\tJoint3_Cmd\tTime\n");
    /*for test*/
    int count = 0; // 验证ros的指令发布频率

    ros::Rate loop_rate(60); // 伺服手频率（125） motor_feedback获得一次舵机位置的时间是0.015s 所以频率改为60Hz

    while (ros::ok())
    {

#ifdef Enable_Finger_L
        /*一次路径插值*/
        arrayD_three endPosL_temp;
        double Time_FingerL_temp = GetOffsetTime(finger_L_);

        for (int i = 0; i < 3; ++i)
        {
            endPosL_temp[i] = Calc1JiTraje((finger_L_.Origin_Pos(i + 1, 1)), (finger_L_.Goal_Pos(i + 1, 1)), Duration_, Time_FingerL_temp);
        }
        /*运动学逆解*/
        MATRIX_D JointThetaL_temp = Inver_kinematics(endPosL_temp, Finger_L);

        /*舵机指令发布*/
        finger_L_.Cmd_publish(JointThetaL_temp);

        // /*舵机状态获取*/
        // finger_L_.Motor_feedback();

        // /*for test*/
        // // cout<<"Finger_L Goal pos:*-*"<<finger_L_.Goal_Pos(1,1)<<"*-*"<<finger_L_.Goal_Pos(2,1)<<"*-*"<<finger_L_.Goal_Pos(3,1)<<endl;
        // hd_servo::JointSpeed msg_L;
        // msg_L.Motor1_Speed=finger_L_.Motor_Speed[0];
        // msg_L.Motor2_Speed=finger_L_.Motor_Speed[1];
        // msg_L.Motor3_Speed=finger_L_.Motor_Speed[2];
        // MotorsL_Speed_pub.publish(msg_L);

        // fprintf(InfoL_EndPos_Joint, "X:%lf\tY:%lf\tA:%lf\tJ1:%lf\tJ2:%lf\tJ3:%lf\tT:%lf\n", endPosL_temp[0], endPosL_temp[1],
        //         endPosL_temp[2], JointThetaL_temp(1, 1), JointThetaL_temp(2, 1), JointThetaL_temp(3, 1), Time_FingerL_temp);

#endif

#ifdef Enable_Finger_R

        /*一次路径插值*/
        arrayD_three endPosR_temp;
        double Time_FingerR_temp = GetOffsetTime(finger_R_);
        for (int i = 0; i < 3; ++i)
        {
            endPosR_temp[i] = Calc1JiTraje((finger_R_.Origin_Pos(i + 1, 1)), (finger_R_.Goal_Pos(i + 1, 1)), Duration_, Time_FingerR_temp);
        }

        /*运动学逆解*/
        MATRIX_D JointThetaR_temp = Inver_kinematics(endPosR_temp, Finger_R);

        /*舵机指令发布*/
        finger_R_.Cmd_publish(JointThetaR_temp);

        /*舵机状态获取*/
        /////////////////////////////////////////////不建议运行导致频率低///////////////////////////////////////////////////////////////
        // finger_R_.Motor_feedback();//舵机状态的获取导致的函数调用会将循环的频率限制在15hz左右，可尝试多线程方式来获取舵机的实时状态
        // /*for test*/
        // // cout<<"Finger_R Goal pos:*-*"<<finger_R_.Goal_Pos(1,1)<<"*-*"<<finger_R_.Goal_Pos(2,1)<<"*-*"<<finger_R_.Goal_Pos(3,1)<<endl;
        // hd_servo::JointSpeed msg_R;
        // msg_R.Motor1_Speed=finger_R_.Motor_Speed[0];
        // msg_R.Motor2_Speed=finger_R_.Motor_Speed[1];
        // msg_R.Motor3_Speed=finger_R_.Motor_Speed[2];
        // MotorsR_Speed_pub.publish(msg_R);
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // fprintf(InfoR_EndPos_Joint, "X:%lf\tY:%lf\tA:%lf\tJ1:%lf\tJ2:%lf\tJ3:%lf\tT:%lf\n", endPosR_temp[0], endPosR_temp[1],
        //         endPosR_temp[2], JointThetaR_temp(1, 1), JointThetaR_temp(2, 1), JointThetaR_temp(3, 1), Time_FingerR_temp);
        // if(Time_FingerR_temp<Duration_){/*测试指令发送频率*/
        //     ++count;
        //     printf("Count：%d\n",count);
        // }
#endif

        /*for test*/
        // cout<<"Finger_R--"<<"theta_1--"<<JointThetaR_temp(1,1)<<"theta_2--"<<JointThetaR_temp(2,1)<<"theta_3--"<<JointThetaR_temp(3,1)<<endl;

        /*for test:根据舵机反馈信息，计算当前末端位姿,并将其发布*/

        /*for test:把插值末端位姿信息发布，观察位姿的跟随情况*/

        ros::spinOnce();
        loop_rate.sleep();
    }
#ifdef Enable_Finger_L
    finger_L_.Finger_end();
#endif
#ifdef Enable_Finger_R
    finger_R_.Finger_end();
#endif

    /*for test*/
    // fclose(InfoL_EndPos_Joint);
    // fclose(InfoR_EndPos_Joint);

    return 0;
}