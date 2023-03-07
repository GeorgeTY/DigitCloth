#!/usr/bin/env python3
#coding=utf-8
from glob import glob
from os import stat
from cv2 import OPTFLOW_USE_INITIAL_FLOW, getTrackbarPos
import matplotlib.pyplot as plt
import numpy as np
import cv2 
import pprint
from digit_interface.digit import Digit
from digit_interface.digit_handler import DigitHandler
from numpy.core.fromnumeric import ptp
import math
import time
from threading import Thread
import datetime

import rospy
from test_hd.msg import state

maniPulate_state = "init_position"
slip_sum = 0
work_sum = 0
down_border = 150
up_border = 380

win_start = 0
win_width = 100

left_border = win_start
right_border = left_border + win_width
pre_left_border = left_border

def init():

    global camera
    global pyr_scale
    global levels
    global winsize
    global iterations
    global poly_n
    global poly_sigma
    global flags

    global ax  # 定义一个 x 轴的空列表用来接收动态的数据
    global ay   # 定义一个 y 轴的空列表用来接收动态的数据
    global t
    camera = cv2.VideoCapture(0) #不使用digit而使用相机时

    # camera = cv2.VideoCapture("/dev/finger_camera") #不使用digit而使用相机时
    pyr_scale=0.5  #金字塔上下两层之间的尺度关系 
    levels=3       #金字塔层数
    winsize=12    #均值窗口大小
    iterations=3   #迭代次数
    poly_n=5       #像素领域大小
    poly_sigma=1.2 #高斯标注差
    flags=0        #计算方法

    ax = []   # 定义一个 x 轴的空列表用来接收动态的数据
    ay = []   # 定义一个 y 轴的空列表用来接收动态的数据
    t=0
    global y_min
    global y_max
    y_min=0
    y_max=3.0
    plt.ion()   # 开启一个画图的窗口
    plt.ylim(y_min,y_max)
    for x in range(100):#初始化画表数据，参数100意味着窗口大小
        ax.append(x*0.1)
        ay.append(0)

    global prvs
    return_value, frame1 = camera.read()
    
    frame1=recut_frame(frame1,down_border,up_border,left_border,right_border)
    prvs = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)

def mkdir(path):
    import os
 		#function：新建文件夹
        #path：str-从程序文件夹要要创建的目录路径（包含新建文件夹名）
    path=path.strip()   #strip方法只要含有该字符就会去除
        #去除首尾\符号
    path=path.rstrip('\\')
        #判断路径是否存在
    isExists = os.path.exists(path)

    if not isExists:
        os.makedirs(path)
        print(path+'创建成功')
        return True
    else:
        print(path+'目录已存在')
        return False

def makePathNow():
    time_tuple_1 = time.localtime()
    bj_time = time.strftime("%Y-%m-%d-%H-%M-%S", time_tuple_1)

    print("开始保存图片")
    i=0
    path="/home/jhz/catkin_ws/src/fingerTip_image/image_data/"+str(bj_time)
    mkdir(path)
    return path
def savePic(path,frame,num):
    filename="/"+str(num)   #/home/jhz/桌面/gelsight_resconstruct/digit-main/digiteye/data_set/texture_exist
    cv2.imwrite(path+ filename + '.jpg', frame)





def recut_frame(frame,top,bottom,left,right):
    width=frame.shape[1]
    height=frame.shape[0]
    print(width, height)
    if height<bottom or left>width:
        print("The size of input is wrong")
    else:
        frame=frame[top:bottom,left:right,:]
    return frame

def initwin_size():
    while True:
        down_border_ = cv2.getTrackbarPos("downBorder_set","parameter_set")
        up_border_ = cv2.getTrackbarPos("upBorder_set","parameter_set")
        return_value, frame = camera.read()
        frame = recut_frame(frame, up_border_, down_border_, 0, 640)
        cv2.imshow("img", frame)
        cv2.waitKey(100)
    

def draw_line(start_pointX,start_pointY,dx,dy):
    magnific_sacle=1
    end_x=start_pointX+dx*magnific_sacle
    end_y=start_pointY+dy*magnific_sacle
    return end_x,end_y

# /*init_position=0,
# force_control,
# init_Pos_after_force,
# turnL_clock/*通过摩擦面带动布料向手内*/,
# moveL_up,/*moveL_up与turnL_anticlock配合，完成接触区域的转换*/
# moveL_down,
# turnL_anticlock,/**/
# stop*/
def draw_plot(val,state):
    global t
    global ax
    global ay
    ax.append(t*0.1)  
    ax.pop(0)
    ay.pop(0)
    t=t+1
    # ay.append(val)
    plt.clf() 
    if state=="turnL_clock":#将转动区间的数据单独颜色标记
        ay.append(val)
        plt.plot(ax,ay,'-r')
    else:
        ay.append(0)
        plt.plot(ax,ay,'-g')
    plt.ylim(y_min,y_max) #限制绘图的上下限制
    plt.draw()
    plt.pause(0.00000001)      

def nothing(x):
    pass

def Callback(data):
    global maniPulate_state
    # print("The current manipulate process is :",data.state)
    maniPulate_state=data.state
    
def worker():
    global left_border
    global y_max
    global pre_left_border
    global prvs
    global win_start
    global down_border 
    global up_border


    rospy.init_node("listen_cloth_state",anonymous=True)

    rospy.Subscriber("Cloth_Maipulation_State",state,Callback,queue_size=1)
    rate=rospy.Rate(10)

    flow_threshold=28
    cv2.namedWindow("parameter_set")
    cv2.createTrackbar("flow_threshold","parameter_set", flow_threshold, 80, nothing)

    y_limit=3
    cv2.createTrackbar("y_max","parameter_set", y_limit, 15, nothing)

    pre_left_border = left_border

    cv2.createTrackbar("winBorder_set","parameter_set", 0, 640 - win_width, nothing)
    left_border = win_start
    right_border = left_border + win_width

    path_name=makePathNow()
    seq_num=0

    prv_state="init"

    # cv2.createTrackbar("downBorder_set","parameter_set", 480, 480, nothing)
    # cv2.createTrackbar("upBorder_set","parameter_set", 0, 480, nothing)
    # initwin_size()

    while not rospy.is_shutdown():

        ### 参数调节
        flow_threshold=cv2.getTrackbarPos("flow_threshold","parameter_set")
        y_limit=cv2.getTrackbarPos("y_max","parameter_set")
        y_max=y_limit

        win_start = cv2.getTrackbarPos("winBorder_set","parameter_set")

        ### 图像读取
        return_value, frame2 = camera.read()
        ### 保存图像
        # savePic(path=path_name,frame=frame2,num=seq_num)
        # seq_num=seq_num+1

        #### 图像裁剪
        left_border = win_start
        right_border = left_border + win_width

        frame2=recut_frame(frame2,down_border,up_border,left_border,right_border)
        print(left_border, right_border)
        # frame2=recut_frame(frame2,down_border,up_border,0,100)

        cv2.imshow('cut_frame2',frame2)
        next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
        cv2.imshow("gray",next)

        ### 判断是否出现了窗口移动

        if (left_border != pre_left_border):
            prvs = next
            pre_left_border = left_border
            continue
        pre_left_border = left_border

 
        ### 光流计算
        flow = cv2.calcOpticalFlowFarneback(prvs,next, None, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags)
        # flow = cv2.calcOpticalFlowFarneback(prvs,next, None, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
        # calcOpticalFlowFarneback（）的参数含义如下：
        # 第一个参数prev：输入的上一帧图像，为8位单通道图；
        # 第二个参数next：输入的当前帧（或者叫下一帧）图像，为8位单通道图；
        # 第三个参数flow：输出的光流矩阵，其尺寸和输入图像一致，矩阵中每个元素都是一个Point2f类型的点，表示在输入图像中相同位置的像素点在上一帧和当前帧图像中分别在x方向和y方向的位移，即（dx，dy）；
        # 第四个参数pyr_scale：生成图像金字塔时上下两层的缩放比例，取值范围是0~1；当该参数为0.5时，即为经典的图像金字塔；
        # 第五个参数level：生成的图像金字塔的层数；当level=0时表示不使用图像金字塔的FB稠密光流算法；一般取level=3；
        # 第六个参数winsize：表示滤波和检测的窗口大小，该参数越大对噪声抑制能力越强，并且能够检测快速移动目标（目标像素点不会移出窗口），但会引起运动区域的模糊；
        # 第七个参数iterations：对每层金字塔图像进行FB算法时的迭代次数；
        # 第八个参数poly_n：对当前像素点进行多项式展开时所选用的邻域大小，该参数值越大，运动区域模糊程度越大，对目标运动检测更稳定，会产生更鲁棒的算法和更模糊的运动场，官方推荐poly_n = 5或7；
        # 第九个参数poly_sigma：进行多项式展开时的高斯系数；推荐值为：当poly_n = 5时，poly_sigma = 1.1；当poly_n = 7时，poly_sigma = 1.5；
        # 第十个参数flag：进行光流估算的滤波器，有以下两种选择：
        # （1）OPTFLOW_USE_INITIAL_FLOW使用输入流作为初始流近似值，并使用盒子滤波器进行光流估算；
        # （2）OPTFLOW_FARNEBACK_GAUSSIAN使用高斯滤波器进行光流估算，高斯滤波器相比盒子滤波器的估算结果更精确，但运行速度较慢。
        mask=np.zeros_like(next)

        res=math.sqrt(math.pow(flow[0].mean(),2)+math.pow(flow[1].mean(),2))
        # print("flow_mean: ",res)
    
        # print("y:",flow[0].mean())

        # draw_plot(flow[0].mean(),maniPulate_state)   
        draw_plot(res,maniPulate_state)   

        global work_sum
        global slip_sum
        # if  (maniPulate_state=="turnL_clock" and abs(flow[0].mean())>(flow_threshold/10)):
        if  (maniPulate_state=="turnL_clock" ):
            work_sum=work_sum+1
            if res>(flow_threshold/10):
                slip_sum=slip_sum+1
        else:
            if prv_state=="turnL_clock":
                print("flow_threshold: ",flow_threshold)
                print("Slip_ratio: ",slip_sum/work_sum)
                work_sum=0
                slip_sum=0

        for i in range(0,flow.shape[0]-1,10):#行，对应y
            for j in range(0,flow.shape[1]-1,10):#列，对应x
  
                (x,y)=(j,i)
                end_X,end_Y=draw_line(x,y,flow[i][j][0],flow[i][j][1])   
                mask=cv2.line(mask, (int(x),int(y)),(int(end_X),int(end_Y)), (255,255,255), 2)

        cv2.imshow("flow_direction",mask)
    
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

        prvs = next
        prv_state=maniPulate_state

        rate.sleep()
    camera.release()

if __name__=="__main__":
    try:
        init()
        worker()
    except rospy.ROSInterruptException:
        pass


