#ifndef KINEMATICS_H
#define KINEMATICS_H
#include<math.h>
#include"INST.h"
#include"matrix.h"


/*定义手指结构参数*/
#define link_1 59.0
#define link_2 59.0
// #define link_3 60.0

/*Digit传感器安装在Finger_L*/
// #define link_3_L 50.0
// #define link_3_R 50.0
// #define DigitSurf_link3_L  -48.0//左手Digit传感器表面相对于link3的偏至距离,以手指坐标系为基准符号
// #define DigitSurf_link3_R  -34.0//右手光滑面相对于link3的偏至距离,以手指坐标系为基准符号

/*Digit传感器安装在Finger_R*/
#define link_3_L 47.0 //48.0                                                                                                                                                                                                                                                                                                                           
#define link_3_R 50.0
#define DigitSurf_link3_L  0.0//-34.0//左手Digit传感器表面相对于link3的偏至距离,以手指坐标系为基准符号
#define DigitSurf_link3_R  -48.0//右手光滑面相对于link3的偏至距离,以手指坐标系为基准符号


#define axis0_center 23.9//20.0 //与基座固连的舵机与中心的偏至距离，轴中心距47.8
// #define DigitSurf_link3  -48.0//-16.0//(-10.0)//Digit传感器表面相对于link3的偏至距离,以手指坐标系为基准符号
#define DegtoRad  (M_PI/180.0)
#define RadtoDeg (180.0/M_PI)


MATRIX_D Inver_kinematics(arrayD_three end_pos,int flag);//手指逆运动学，输入末端位姿，理论关节角θ（°），θ主要用于正逆运动学计算，传给舵机的关节指令还需要进行转换。
MATRIX_D Forward_kinematics(arrayD_three joint_pos,int flag);//手指正运动学。输入θ角，得到末端位姿(相对于总的手基坐标系)

/*正逆运动学写成齐次矩阵形式，只负责计算从手指第一个转轴到末端接触点的*/

#endif