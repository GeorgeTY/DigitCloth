#!/usr/bin/env python3
#coding=utf-8
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
MAX_INT=sys.maxsize
MIN_INT= -sys.maxsize - 1

#############实时采集图像################
# video1 = cv2.VideoCapture("/dev/finger_camera")
video1 = cv2.VideoCapture("/dev/fig_cam")
width = (int(video1.get(cv2.CAP_PROP_FRAME_WIDTH)))
height = (int(video1.get(cv2.CAP_PROP_FRAME_HEIGHT)))
ret,frame=video1.read()


def nothing(x):
    pass

def recut_frame(frame,bottom_Y,top_Y,left_X,right_X):
    width=frame.shape[1]
    height=frame.shape[0]
    if height<bottom_Y or left_X>width:
        print("The size of input is wrong")
    else:
        frame=frame[top_Y:bottom_Y,left_X:right_X,:]
        # print(frame.shape)
    return frame


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

def find_blue(frame,threshold):
    (b,g,r)=cv2.split(frame)
    cv2.imshow("gray",b)
    # cv2.imshow("green channel:",g)
    # cv2.imshow("red channel:",r)
    # ret,binary=cv2.threshold(g,0,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)
    # ret,binary = cv2.threshold(b,threshold,255,cv2.THRESH_BINARY) 
    ret,binary = cv2.threshold(b,threshold,255,cv2.THRESH_BINARY) 


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


def talker():

    pub = rospy.Publisher('Detect_cloth', clothEdge, queue_size=1)

    rospy.init_node('talker1', anonymous=True)
    #启动节点同时为节点命名， 若anoymous为真则节点会自动补充名字，实际名字以talker_322345等表示
    #若为假，则系统不会补充名字，采用用户命名。但是一次只能有一个同名节点，若后面有一个相同listener
    #名字的节点则后面的节点启动会注销前面的相同节点名。
    
    rate = rospy.Rate(10) # 10hz
    #延时的时间变量赋值，通过rate.sleep()实现延时
    left_X=560
    binary_threshold=140
    pixels_numbers_threahold=17000#7500
    cv2.namedWindow("gray_set")
    cv2.createTrackbar("binary_threshold","gray_set", binary_threshold, 255, nothing)
    cv2.createTrackbar("pixels_numbers_threahold","gray_set", pixels_numbers_threahold, 40000, nothing)
    cv2.createTrackbar("frame_recut_threahold","gray_set", left_X, 630, nothing)

    cv2.namedWindow("gray",0)
    cv2.namedWindow("binary",0)
    cv2.namedWindow("original",0)
    cv2.resizeWindow("gray",160,540)
    cv2.resizeWindow("binary",160,540)
    cv2.resizeWindow("original",160,540)




    msg=clothEdge()
    while not rospy.is_shutdown():
        binary_threshold=cv2.getTrackbarPos("binary_threshold","gray_set")
        pixels_numbers_threahold=cv2.getTrackbarPos("pixels_numbers_threahold","gray_set")
        left_X=cv2.getTrackbarPos("frame_recut_threahold","gray_set")

        ret,frame=video1.read()
        # frame=recut_frame(frame=frame,bottom_Y=frame.shape[0]-1,top_Y=0,left_X=300,right_X=frame.shape[1]-1) #贴在斜面上的安装位置
        frame=recut_frame(frame=frame,bottom_Y=frame.shape[0]-1,top_Y=0,left_X=left_X,right_X=640)

        cv2.imshow("original",frame)
        # binary=find_green(frame,binary_threshold)
        binary=find_blue(frame,binary_threshold)
        # binary=find_red(frame,binary_threshold)
        # binary=find_gray(frame,binary_threshold)
        # binary[binary==255] = 1
        binary[binary==0] = 0
        binary[binary==255] = 1
     
        # Blackpiexls_nums=frame.shape[0]*frame.shape[1]-binary.sum()
        Blackpiexls_nums=binary.sum()
       
        print(int(Blackpiexls_nums))

        if(Blackpiexls_nums>pixels_numbers_threahold):
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
        talker()
    except rospy.ROSInterruptException:
        pass
# while True:
#     time.sleep(1)
#     height_array=array_show(digit_R)
#     print(height_array)