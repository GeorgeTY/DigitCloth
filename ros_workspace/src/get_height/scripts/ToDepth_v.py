#!/usr/bin/env python3
#coding=utf-8
import rospy
from numpy.core.fromnumeric import reshape
import cv2
import img_to_depth_digit as itd
import time
import sys
import pprint
from digit_interface.digit import Digit
from digit_interface.digit_handler import DigitHandler
import numpy as np
from get_height.msg import Height
from rospy import msg

width=640
height=480
force_threshold=0.05
pointcloud = None
pcdm = []

# R_digit_folder="/home/jhz/catkin_ws/src/get_height/digiteye/R_calibrate_ballR2" #digit校准文件所在文件夹 每个digit都不同

R_digit_folder="/home/jhz/catkin_ws/src/get_height/digiteye/calibrate_new"
def digit_connect():
    # Print a list of connected DIGIT's
    digits = DigitHandler.list_digits()
    print("Connected DIGIT's to Host:")
    pprint.pprint(digits)
    print(digits[0]['serial'])
    digit= Digit(digits[0]['serial'], "Right Gripper")
    digit.connect()
    print(digit.info())
    
    intensity=15
    # red=(intensity,0,0)
    # green=(0,intensity,0)
    # blue=(0,0,intensity)
    rgb=(intensity,intensity,intensity)
    light_color=rgb
    digit.set_intensity_rgb(*light_color)

    vga_res = Digit.STREAMS["VGA"]
    digit.set_resolution(vga_res)
    # Change DIGIT FPS to 15fps
    fps_15 = Digit.STREAMS["VGA"]["fps"]["15fps"]
    digit.set_fps(fps_15)
    return digit

itd_cvter_R = itd.ImageToDepth(R_digit_folder)
digit_R= digit_connect()


def get_frame(which_digit):
    frame=which_digit.get_frame(transpose=True)
    return frame

def array_show(which_digit,flag_=0): #将高度区域分为16*12阵列
    frame=get_frame(which_digit)
    frame = cv2.resize(frame, (int(width), int(height)), interpolation=cv2.INTER_CUBIC)
    image=frame
    cv2.imshow("digit",image) #_____________  后加用于录传感器图像____________________#
    cv2.waitKey(2)
    if which_digit==digit_R:
        depth, hm ,ImgGrad = itd_cvter_R.convert(frame)

    #height_array 的形状与显示的图片一致，是横放的，col_num决定了width长度，即横向被划分为几个区域；lin_num决定为了height长度，即竖向被划分为几个区域
    wid=hm.shape[0] 
    hei=hm.shape[1]
    lin_num=3
    col_num=6
    if flag_==0:
        lin_num=3
        col_num=6
    elif flag_==1:
        lin_num=12
        col_num=16
        
    height_array=np.ones((lin_num,col_num))

    cols=np.arange(0,wid+0.01,wid//lin_num)  #因为np.arrange(start,stop,step)范围不包括stop,所以加0.01
    lins=np.arange(0,hei+0.01,hei//col_num)
    cols=(cols.astype(np.int16))
    lins=(lins.astype(np.int16))

    for i in range(0,cols.shape[0]-1):
        for j in range(0,lins.shape[0]-1):
            slice=hm[cols[i]:cols[i+1],lins[j]:lins[j+1]]
            mean=slice.mean()
    
            if mean<force_threshold:
                height_array[i][j]=0.0
            else:
                height_array[i][j]=mean
      
    # print(height_array)
    return height_array

def talker():
    pub = rospy.Publisher('Rdigit_height', Height, queue_size=10)
 
    rospy.init_node('talker_R', anonymous=True)
 
    rate = rospy.Rate(10) # 10hz 
    
    while not rospy.is_shutdown():

        msg=Height()
        height_array=array_show(digit_R)

        #在运行的terminal界面info 出信息，可不加，可随意改
        print(height_array)
     
        msg.which_digit="Digit_R"
        msg.height_data0[0]=height_array[0][5]
        msg.height_data0[1]=height_array[1][5]
        msg.height_data0[2]=height_array[2][5]
        msg.height_data1[0]=height_array[0][4]
        msg.height_data1[1]=height_array[1][4]
        msg.height_data1[2]=height_array[2][4]
        msg.height_data2[0]=height_array[0][3]
        msg.height_data2[1]=height_array[1][3]
        msg.height_data2[2]=height_array[2][3]
        msg.height_data3[0]=height_array[0][2]
        msg.height_data3[1]=height_array[1][2]
        msg.height_data3[2]=height_array[2][2]
        msg.height_data4[0]=height_array[0][1]
        msg.height_data4[1]=height_array[1][1]
        msg.height_data4[2]=height_array[2][1]
        msg.height_data5[0]=height_array[0][0]
        msg.height_data5[1]=height_array[1][0]
        msg.height_data5[2]=height_array[2][0]


        
        pub.publish(msg)
        #发布数据 必须发布
        
        rate.sleep()
        #ros中的延时表示，也可用系统的延时 import time  time.sleep(1) 
        #ros系统中的延时应该比ubuntu自带好好，尽量用ros的

if __name__ == '__main__':

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
