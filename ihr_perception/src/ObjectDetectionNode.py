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

        # Set up a subscriber, publisher and CvBridge object
        self.sub=rospy.Subscriber('/cameras/right_hand_camera/image',Image, self.callback)
        self.img_pub = rospy.Publisher('gaussian', Image, queue_size=10)
        self.hum_pub = rospy.Publisher('hum1',data_class=Pose,queue_size=10)
    def callback(self, data):
        rospy.loginfo("Data received")
        try:
            cv_image=bridge.imgmsg_to_cv2(data,"bgr8")
            #3)
            gaussian=cv2.GaussianBlur(cv_image,(5,5),0)
            #4)
            hsv=cv2.cvtColor(gaussian,cv2.COLOR_BGR2HSV)
            lower_red=np.array([0,30,30])
            upper_red=np.array([10,255,255])
            mask=cv2.inRange(hsv, lower_red, upper_red)
            res=cv2.bitwise_and(gaussian,gaussian,mask=mask)

            res_gray=cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)

            #_, mask=cv2.threshold(res_gray,127,255,0)
            #Red Rectangle static
            #5)
            contours,hierachy=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            #6)
            for i in contours:
                x,y,w,h=cv2.boundingRect(i)
                if cv2.contourArea(i) >1400:
                    img_rect=cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,0,255),2)
                    moments=cv2.moments(i,False)
                    humoments=cv2.HuMoments(moments)
                    self.humoment1list.append(humoments[0])
                    self.humoment2list.append(humoments[1])
            if(self.plot):
                pose_msg=Pose()
                pose_msg.position.x=self.humoment1list[0]
            #6)
            # for i in contours:
            #     ((x,y),(w,h),a)=cv2.minAreaRect(i)
            #     if cv2.contourArea(i) >200:
            #         img_rect=cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,0,255),2)
            #         #img_cont=cv2.drawContours(cv_image,i,-1,(255,0,0),2)

            #for i in contours:
            #    ((x,y),(w,h),angle)=cv2.minAreaRect(i)
            #img=cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,0,255),3)
            #img=cv2.drawContours(cv_image, contours,-1,(0,255,0),2)
            ros_img=bridge.cv2_to_imgmsg(img_rect,"bgr8")
            #cv2.imshow('Test',ros_img)
            self.img_pub.publish(ros_img)
            self.hum_pub.publish(pose_msg)

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
        choices=['red', 'green', 'blue', 'yellow'],
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
