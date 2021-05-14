#!/usr/bin/env python
import rospy
import rospkg
import os
import sys
import cv2
import cv_bridge

from your_ros_intro.srv import *


from sensor_msgs.msg import Range, Image

def number_client(x):
    '''
    wait for the service and hand over the threshold 
    '''
    ##################
    # YOUR CODE HERE #
    ##################

def usage():
    return "%s [x]"%sys.argv[0]

if __name__ == '__main__':
    '''
    Get the new threshold from the screen and start the handover 
    '''
    # asking the user which heigh should be used 
    if len(sys.argv) == 2:
        x = float(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)

    print("Changing Threshold %s"%x)
    # start the handover 
    number_client(x)