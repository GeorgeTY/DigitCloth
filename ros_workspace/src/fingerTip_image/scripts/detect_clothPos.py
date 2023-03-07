#!/usr/bin/env python3
#coding=utf-8
import imp
from cv2 import threshold
from matplotlib import image

from urllib3 import Retry
import rospy
import cv2
import time
import numpy as np
from rospy import msg

from numpy import *
import numpy as np
import time
from skimage import morphology
import sys
from fingerTip_image.msg import clothEdge
from test_hd.msg import state
from std_msgs.msg import Header
from sensor_msgs.msg import Image
MAX_INT=sys.maxsize
MIN_INT= -sys.maxsize - 1

#####################实时采集图像#######################
#!!!!!注意灯光电压调至2.7V因为灯光亮度对于二值化影响较大!!!!!#
# video1 = cv2.VideoCapture("/dev/finger_camera")
cam_num=0

# width = (int(video1.get(cv2.CAP_PROP_FRAME_WIDTH)))
# height = (int(video1.get(cv2.CAP_PROP_FRAME_HEIGHT)))
# ret,frame=video1.read()

def calc_gray(frame,threshold):
    (b,g,r)=cv2.split(frame)
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    ret,binary=cv2.threshold(g,0,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)
    # ret,binary = cv2.threshold(gray,threshold,255,cv2.THRESH_BINARY) 
    # cv2.imshow("binary",binary)
    return binary

def pixelsInWindow(frame,winStart,winEnd):
   #計算窗口內的紋理點的數量
    window=frame[:,winStart:winEnd]
    window[window==255]=1
    pixelNum=window.sum()
    print("Pixels_num is:",pixelNum)
    # print(window.shape) 
    return pixelNum


def nothing(x):
    pass

def pub_imag(image_pub,imagedata):
    image_temp=Image()
    header=Header(stamp=rospy.Time.now())
    header.frame_id='map'
    image_temp.height=480
    image_temp.width=640
    image_temp.encoding='rgb8'
    image_temp.data=np.array(imagedata).tostring()
    image_temp.header=header
    image_temp.step=640*3
    image_pub.publish(image_temp)



def worker():
    video=cv2.VideoCapture(cam_num)
    retval = cv2.getGaborKernel(ksize=(111,111), sigma=2, theta=70, lambd=70, gamma=1.2)
    binary_threshold=1

    #可以佔時使用cloth_edge這個消息來表示，不過，只有當檢車到布料剛好出現在視野中時，才進行發送true消息
    pub = rospy.Publisher('cloth_Edge', clothEdge, queue_size=1)
    image_pub=rospy.Publisher('fingeImage',Image,queue_size=1)
    
    rospy.init_node('talker', anonymous=True)
 
    rate = rospy.Rate(10) # 10hz
    #延时的时间变量赋值，通过rate.sleep()实现延时
    # right_X=90
    # binary_threshold=55
    NumPixels_threshold=2500
    cv2.namedWindow("NumPixels_threshold_set")
    cv2.createTrackbar("NumPixels_threshold","NumPixels_threshold_set", NumPixels_threshold, 20000, nothing)

    # cv2.namedWindow("gray",0)
    # cv2.resizeWindow("gray",640,480)

    Surpass_threshold_count=0 #如果连续10个周期检测到超过阈值，则发出检测到布料边缘的消息；

    msg=clothEdge()
    while not rospy.is_shutdown():
        NumPixels_threshold=cv2.getTrackbarPos("NumPixels_threshold","NumPixels_threshold_set")

        ret,src = video.read()
        pub_imag(image_pub,src)
        cv2.imshow("src:",src)
        cv2.waitKey(2)

        gaborFilted = cv2.filter2D(src,-1,retval)
        cv2.imshow("gaborFilted:",gaborFilted)

        binary=calc_gray(gaborFilted,binary_threshold)
        cv2.imshow("binary:",binary)

            ## b.设置卷积核5*5
        kernel = np.ones((5,5),np.uint8)
            
            ## c.图像的腐蚀，默认迭代次数
        erosion = cv2.erode(binary,kernel)
            ## 腐蚀后
        cv2.imshow('after fir_erosion',erosion)


        kernel = np.ones((25,5),np.uint8)
            ## 图像的膨胀
        dilated = cv2.dilate(erosion,kernel)
            ## 膨胀后
        cv2.imshow('after fir_dilate',dilated )

        NumofPixels=pixelsInWindow(frame=dilated,winStart=600,winEnd=639)



        if NumofPixels>NumPixels_threshold:
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

# 因爲布料的紋理很強，在指間沒有布料時，相機看到的圖像是固定的（大概就是digit）,所以通過gabor紋理檢測後，
# 檢查視野中靠近砂紙部分是否有足夠的紋理特特徵，當前一步紋理特徵存在，而後在調整過程中不存在時，即認爲布料褶皺高度剛好被檢車到