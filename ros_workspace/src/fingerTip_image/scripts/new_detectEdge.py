#!/usr/bin/env python3
#coding=utf-8
import imp

from cv2 import imread
import rospy
import cv2
import time
import numpy as np
from rospy import msg

from pickle import FRAME
# from visualization.matplot.helper import PIXEL_TO_INCH
import cv2
# import img_to_depth_digit as itd
from numpy import *
import numpy as np
import time
from skimage import morphology
import sys
from fingerTip_image.msg import clothEdge
from test_hd.msg import state
MAX_INT=sys.maxsize
MIN_INT= -sys.maxsize - 1

#####################实时采集图像#######################
#!!!!!注意灯光电压调至2.7V因为灯光亮度对于二值化影响较大!!!!!#
# video1 = cv2.VideoCapture("/dev/finger_camera")
video1 = cv2.VideoCapture(0)

# video1 = cv2.VideoCapture(0)

width = (int(video1.get(cv2.CAP_PROP_FRAME_WIDTH)))
height = (int(video1.get(cv2.CAP_PROP_FRAME_HEIGHT)))
ret,frame=video1.read()

##检测边缘的时机，亚克力平移阶段与转动阶段，以下为各状态，检测时机初步设定为moveL_up与turnL_clock
            # init_position=0,
            # force_control,
            # init_Pos_after_force,
            # turnL_clock/*通过摩擦面带动布料向手内*/,
            # moveL_up,/*moveL_up与turnL_anticlock配合，完成接触区域的转换*/
            # moveL_down,
            # turnL_anticlock,/**/
            # stop
maniPulate_state="init_position"


def nothing(x):
    pass

def find_green(frame,threshold):
    (b,g,r)=cv2.split(frame)
    # cv2.imshow("blue channel:",b)
    cv2.imshow("gray",g)
    # cv2.imshow("red channel:",b)
    # ret,binary=cv2.threshold(g,0,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)
    ret,binary = cv2.threshold(g,threshold,255,cv2.THRESH_BINARY) 
    cv2.imshow("binary",binary)
    return binary

def find_red(frame,threshold):
    (b,g,r)=cv2.split(frame)
    # cv2.imshow("blue channel:",b)
    # cv2.imshow("green channel:",g)
    cv2.imshow("gray",r)
    # ret,binary=cv2.threshold(g,0,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)
    ret,binary = cv2.threshold(r,threshold,255,cv2.THRESH_BINARY) 
    cv2.imshow("binary",binary)
    return binary

def find_gray(frame,threshold):
    # (b,g,r)=cv2.split(frame)
    # cv2.imshow("blue channel:",b)
    # cv2.imshow("green channel:",g)
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    # cv2.imshow("red channel:",r)
    cv2.imshow("gray",gray)

    # ret,binary=cv2.threshold(g,0,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)
    ret,binary = cv2.threshold(gray,threshold,255,cv2.THRESH_BINARY) 
    cv2.imshow("binary",binary)
    return binary

def Callback(data):
    global maniPulate_state
    # print("The current manipulate process is :",data.state)
    maniPulate_state=data.state

def worker():

    pub = rospy.Publisher('cloth_Edge', clothEdge, queue_size=1)

    rospy.init_node('talker', anonymous=True)
    #启动节点同时为节点命名， 若anoymous为真则节点会自动补充名字，实际名字以talker_322345等表示
    #若为假，则系统不会补充名字，采用用户命名。但是一次只能有一个同名节点，若后面有一个相同listener
    #名字的节点则后面的节点启动会注销前面的相同节点名。
    rospy.Subscriber("Cloth_Maipulation_State",state,Callback,queue_size=1)
    
    rate = rospy.Rate(10) # 10hz
    #延时的时间变量赋值，通过rate.sleep()实现延时
    right_X=90
    binary_threshold=55
    pixels_numbers_threahold=20000
    cv2.namedWindow("gray_set")
    cv2.createTrackbar("binary_threshold","gray_set", binary_threshold, 255, nothing)
    cv2.createTrackbar("pixels_numbers_threahold","gray_set", pixels_numbers_threahold, 20000, nothing)
    cv2.createTrackbar("frame_recut_threahold","gray_set", right_X, 630, nothing)

    cv2.namedWindow("gray",0)
    cv2.namedWindow("binary",0)
    cv2.namedWindow("original",0)
    cv2.resizeWindow("gray",640,480)
    cv2.resizeWindow("binary",640,480)
    cv2.resizeWindow("original",640,480)

    Surpass_threshold_count=0 #如果连续10个周期检测到超过阈值，则发出检测到布料边缘的消息；

    msg=clothEdge()
    while not rospy.is_shutdown():
        binary_threshold=cv2.getTrackbarPos("binary_threshold","gray_set")
        pixels_numbers_threahold=cv2.getTrackbarPos("pixels_numbers_threahold","gray_set")
        right_X=cv2.getTrackbarPos("frame_recut_threahold","gray_set")

        # ret,frame=video1.read()
        frame=imread("/home/jhz/桌面/opencv_ws/calc_distance/tst1.jpg")
        cv2.imshow("original",frame)

        binary=find_green(frame,binary_threshold)
        # binary=find_red(frame,binary_threshold)
        # binary=find_gray(frame,binary_threshold)
        binary[binary==255] = 1
        
        Blackpiexls_nums=frame.shape[0]*frame.shape[1]-binary.sum()

        print(int(Blackpiexls_nums))

        if(Blackpiexls_nums>pixels_numbers_threahold and (maniPulate_state=="turnL_clock" or maniPulate_state=="moveL_up")):
            Surpass_threshold_count=Surpass_threshold_count+1
        else:
            Surpass_threshold_count=0

        if Surpass_threshold_count>10:
            clothEdge.edge_inView=True
            print("Detected Cloth Edge")
        else:
            clothEdge.edge_inView=False

        pub.publish(msg)
        #发布数据 必须发布
        
        rate.sleep()
        #ros中的延时表示，也可用系统的延时 import time  time.sleep(1) 
        #ros系统中的延时应该比ubuntu自带好好，尽量用ros的
        if cv2.waitKey(1)==25:
            break

    cv2.destroyAllWindows()
#加一个释放video1资源的语句

if __name__ == '__main__':
    try:
        worker()
    except rospy.ROSInterruptException:
        pass
# while True:
#     time.sleep(1)
#     height_array=array_show(digit_R)
#     print(height_array)