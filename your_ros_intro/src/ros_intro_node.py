#!/usr/bin/env python
import rospy
import rospkg
import os
import sys
import cv2
import cv_bridge
from your_ros_intro import *
from sensor_msgs.msg import Range, Image
class RosIntroNode(object):
    def __init__(self, x):

        self.threshold=x
        # get filesystem path of current package.
        r = rospkg.RosPack()
        pkg_path = r.get_path('your_ros_intro')

        # load success image
        img = cv2.imread(os.path.join(pkg_path, "images/success.jpg"))
        self.success = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")

        # load blank image
        img = cv2.imread(os.path.join(pkg_path, "images/blank.jpg"))
        self.blank = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")

        # Define a Publisher that publishes the image to the /image topic

        self.img_pub = rospy.Publisher('chatter', Image, queue_size=10)

        # init node
        rospy.init_node('listener', anonymous=True)

        # define self.sub to subscribe to the height (range) of the gripper and invoke the callback method

        self.sub = rospy.Subscriber('robot/range/right_hand_range/state', Range , self.callback)


    def callback(self, data):
        #print(intro_node.sub)
        #rospy.loginfo('Received %s', data.range)
        self.range=data.range
        print(self.range)
        if self.range < self.threshold:
            print('success')
            self.img_pub.publish(self.success)
        else:
            print("blank")
            self.img_pub.publish(self.blank)
def change_threshold_service():
    s = rospy.Service('change_threshold',x,handle_change_threshold)#hier stimmt das x nicht
    rospy.spin()
def handle_change_threshold():
    intro_node=RosIntroNode(srv.x)#hier auch nicht
    return numberResponse()
if __name__ == "__main__":
    change_threshold_service()
