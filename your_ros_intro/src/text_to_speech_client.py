#!/usr/bin/env python

import sys
import rospy
from ihr_sdk.srv import *

def call_service(x):
    pass
    #TODO code here


if __name__ == "__main__":
    if len(sys.argv) == 2:
        x = sys.argv[1]
    else:
        sys.exit(1)

    print call_service(x)
