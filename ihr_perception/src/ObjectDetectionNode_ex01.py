#!/usr/bin/env python

import argparse
import rospy
import numpy as np
import cv2
import cv_bridge
#import baxter_interface
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge.core import CvBridge
import matplotlib.pyplot as plt

bridge=cv_bridge.CvBridge()

class Node(object):

    def __init__(self, arm_id, color,plot):
        self.arm_id = arm_id
        self.color = color
        self.plot = plot
        self.humoment1list = []
        self.humoment2list = []
        #2)Set up a subscriber, publisher and CvBridge object
        self.sub=rospy.Subscriber('/cameras/right_hand_camera/image',Image, self.callback)
        self.img_pub = rospy.Publisher('gaussian', Image, queue_size=10)
        #self.hum_pub = rospy.Publisher('hum1',data_class=Pose,queue_size=10)
    def callback(self, data):
        rospy.loginfo("Data received")
        try:
            cv_image=bridge.imgmsg_to_cv2(data,"bgr8")
            img_rect=cv_image
            #3)smoothing the input image
            gaussian=cv2.GaussianBlur(cv_image,(5,5),0)
            #4)filter for red
            hsv=cv2.cvtColor(gaussian,cv2.COLOR_BGR2HSV) #converting to HSV
            lower_red=np.array([0,55,55])               #defining the color spaces
            upper_red=np.array([12,255,255])
            lower_yellow=np.array([16,100,55])               #defining the color spaces
            upper_yellow=np.array([25,255,255])
            lower_green=np.array([40,55,30])               #defining the color spaces
            upper_green=np.array([100,255,255])
            mask_red=cv2.inRange(hsv, lower_red, upper_red)
            #7)Extended Program for Green, Yellow and Debug Detection
            mask_green=cv2.inRange(hsv, lower_green, upper_green)
            mask_yellow=cv2.inRange(hsv, lower_yellow, upper_yellow)
            mask_all=mask_red+mask_yellow+mask_green
            if self.color=='red':
                res=cv2.bitwise_and(gaussian,gaussian,mask=mask_red)#creating output with red only
                cv2.imshow('Test',res)
                #5)contour search
                contours,hierachy=cv2.findContours(mask_red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)#creating contours
            elif self.color=='green':
                res=cv2.bitwise_and(gaussian,gaussian,mask=mask_green)#creating output with red only
                cv2.imshow('Test',res)
                #5)contour search
                contours,hierachy=cv2.findContours(mask_green,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)#creating contours
            elif self.color=='yellow':
                res=cv2.bitwise_and(gaussian,gaussian,mask=mask_yellow)#creating output with red only
                cv2.imshow('Test',res)
                #5)contour search
                contours,hierachy=cv2.findContours(mask_yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)#creating contours
            elif self.color=='all':
                res=cv2.bitwise_and(gaussian,gaussian,mask=mask_all)#creating output with red only
                #cv2.imshow('Test',res)
                contours,hierachy=cv2.findContours(mask_all,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)#creating contours
            #6)filtering for big contours
            for i in contours:
                #x,y,w,h=cv2.boundingRect(i)
                #8) getting the angle
                ((x,y),(w,h),angle)=cv2.minAreaRect(i)
                print(angle)
                if cv2.contourArea(i) >1200:
                    #img_rect=cv2.rectangle(cv_image,(int(x),int(y)),(int(x)+int(w),int(y)+int(h)),(0,0,255),2)
                    img_rect=cv2.drawContours(cv_image,i,-1,(0,0,255),2)
            #         moments=cv2.moments(i,False)
            #         humoments=cv2.HuMoments(moments)
            #         self.humoment1list.append(humoments[0])
            #         self.humoment2list.append(humoments[1])
            # # if(self.plot):
            #     pose_msg=Pose()
            #     pose_msg.position.x=self.humoment1list[0]
            #6)
            # for i in contours:
            #     ((x,y),(w,h),a)=cv2.minAreaRect(i)
            #     if cv2.contourArea(i) >200:
            #         img_rect=cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,0,255),2)
            #         #img_cont=cv2.drawContours(cv_image,i,-1,(255,0,0),2)



            #img=cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,0,255),3)
            #img=cv2.drawContours(cv_image, contours,-1,(0,255,0),2)
            ros_img=bridge.cv2_to_imgmsg(img_rect,"bgr8")#using the vcvbridge in the opposite direction
            #3)
            self.img_pub.publish(ros_img)
            #self.hum_pub.publish(pose_msg)
            cv2.waitKey(1)
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)
        # color bounds
        bounds = {
            'red' : [np.array([0,55,55]),np.array([12,255,255])],
            'yellow' : [np.array([16,100,55]),np.array([25,255,255])],
            'green' : [np.array([40,55,30]),np.array([100,255,255])],
            'blue' : [np.array([105,55,55]),np.array([160,255,255])]
        }



def main():

    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        "-a", "--arm", dest="arm", default="",
        choices=['left', 'right'],
        help="The arm which is used for object detection",
        required=True
    )
    required.add_argument(
        "-c", "--color", dest="color", default="",
        choices=['red', 'green', 'blue', 'yellow','all'],
        help="The color which is searched for",
        required=True
    )
    required.add_argument(
        "-p", "--plot", dest="plot", default="false",
        choices=['true', 'false'],
        help="If plot is enabled",
        required=False
    )

    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("detect_object_"+args.arm, anonymous=True)

    node = Node(args.arm, args.color,args.plot)

    rospy.spin()

if __name__ == "__main__":
    main()
