#!/usr/bin/env python3
#coding=utf-8
import time
from threading import Thread
import datetime

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
video1 = cv2.VideoCapture("/dev/finger_camera")

width = (int(video1.get(cv2.CAP_PROP_FRAME_WIDTH)))
height = (int(video1.get(cv2.CAP_PROP_FRAME_HEIGHT)))
ret,frame=video1.read()

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

def savePic():
    time_tuple_1 = time.localtime()
    bj_time = time.strftime("%Y-%m-%d-%H-%M-%S", time_tuple_1)

    print("开始保存图片")
    i=0
    path="/home/jhz/catkin_ws/src/fingerTip_image/camera_pics/"+str(bj_time)
    mkdir(path)
    while 1:
     
        cv2.waitKey(1)

        ret,frame=video1.read()
        cv2.imshow("original", frame)

        filename="/"+str(i)   #/home/jhz/桌面/gelsight_resconstruct/digit-main/digiteye/data_set/texture_exist
        cv2.imwrite(path+ filename + '.jpg', frame)
        i=i+1


def talker():
  

    pub = rospy.Publisher('New_camera_view', clothEdge, queue_size=1)

    rospy.init_node('talker1', anonymous=True)
    #启动节点同时为节点命名， 若anoymous为真则节点会自动补充名字，实际名字以talker_322345等表示
    #若为假，则系统不会补充名字，采用用户命名。但是一次只能有一个同名节点，若后面有一个相同listener
    #名字的节点则后面的节点启动会注销前面的相同节点名。
    
    rate = rospy.Rate(10) # 10hz
    #延时的时间变量赋值，通过rate.sleep()实现延时
    while not rospy.is_shutdown():
      
        # ret,frame=video1.read()
    
        # cv2.imshow("original",frame)
    
        rate.sleep()
 
        # if cv2.waitKey(1)==25:
        #     break

    cv2.destroyAllWindows()
#加一个释放video1资源的语句



if __name__ == '__main__':
    savePic_T=Thread(target=savePic, args=())
    savePic_T.start()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
# while True:
#     time.sleep(1)
#     height_array=array_show(digit_R)
#     print(height_array)