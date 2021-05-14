#!/usr/bin/env python
import rospy
from your_ros_intro import *
def adds(req):
    return number.srv.x
def adds_server():
    rospy.init_node("adds_server")
    s=rospy.Service('adds',number.srv.x,adds)
    rospy.loginfo('Service Test Running')
    rospy.spin()
