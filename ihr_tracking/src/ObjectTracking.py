#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import rospy
import numpy as np
import cv2
import cv_bridge
#import baxter_interface
from sensor_msgs.msg import Image, Range
from cv_bridge.core import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
from scipy.spatial import distance as dist
from collections import OrderedDict
import numpy as np

#2)
bridge=cv_bridge.CvBridge()

class CentroidTracker(object):

    def __init__(self, camera, color, maxDisappeared=50):

        # initialize the next unique object ID along with two ordered
        # dictionaries used to keep track of mapping a given object
        # ID to its centroid and number of consecutive frames it has
        # been marked as "disappeared", respectively
        self.color = color
        self.nextObjectID = 0
        self.objects = OrderedDict()
        self.disappeared = OrderedDict()
        # store the number of maximum consecutive frames a given
        # object is allowed to be marked as "disappeared" until we
        # need to deregister the object from tracking
        self.maxDisappeared = maxDisappeared
        #2)
        self.sub=rospy.Subscriber('/cameras/right_hand_camera/image',Image, self.callback)
        self.publisherContour = rospy.Publisher('tracking', Image, queue_size=10)


    def detect(self, frame):
        #TODO: Write a function that detect the Object and return the cornerstones of the bounding boxes
        # Use cv2.minAreaRect() as in the previous exercise.
        # A single box should have the following format box = [x,y]
        boxes = []
        #3)
        gaussian=cv2.GaussianBlur(frame,(5,5),0)
        hsv=cv2.cvtColor(gaussian,cv2.COLOR_BGR2HSV) #converting to HSV
        lower_red=np.array([0,55,55])               #defining the color spaces
        upper_red=np.array([12,255,255])
        mask_red=cv2.inRange(hsv, lower_red, upper_red)
        res=cv2.bitwise_and(gaussian,gaussian,mask=mask_red)#creating output with red only
        contours,hierachy=cv2.findContours(mask_red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)#creating contours
        for i in contours:
            #x,y,w,h=cv2.boundingRect(i)
            ((x,y),(w,h),angle)=cv2.minAreaRect(i)
            if cv2.contourArea(i) >1200:
                #img_rect=cv2.rectangle(cv_image,(int(x),int(y)),(int(x)+int(w),int(y)+int(h)),(0,0,255),2)
                img_rect=cv2.drawContours(frame,i,-1,(0,0,255),2)
                boxes.append([x,y])
        #img=cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,0,255),3)
        #img=cv2.drawContours(cv_image, contours,-1,(0,255,0),2)
        #rospy.loginfo("Boxes: ",str(boxes))

        return boxes

    def callback(self,data):
        rospy.loginfo("Data received")
        # convert image from ROS message format to OpenCV
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        frame = cv_image
        #frame = cv2.resize(cv_image, (400,400))                       # why to resize the image?
        rects = []
        #rect = self.detect(greenLower, greenUpper, frame)
        #for i in range(len(rect)):
        #    rects.append(rect[i])

        rect = self.detect(frame)
        for i in range(len(rect)):
            rects.append(rect[i])

        # update our centroid tracker using the computed set of bounding
        # box rectangles
        objects = self.update(rects)
        # loop over the tracked objects
        for (objectID, centroid) in objects.items():
            # draw both the ID of the object and the centroid of the
            # object on the output frame
            text = "ID {}".format(objectID)
            cv2.putText(frame, text, (centroid[0] - 20, centroid[1] - 20), # centroid[0] - 10, centroid[1] - 10
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

        # publish contour image
        imgMsg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisherContour.publish(imgMsg)


    def register(self, centroid):
        # when registering an object we use the next available object
        # ID to store the centroid
        self.objects[self.nextObjectID] = centroid
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID += 1

    def deregister(self, objectID):
        # to deregister an object ID we delete the object ID from
        # both of our respective dictionaries
        del self.objects[objectID]
        del self.disappeared[objectID]

    def connectObj(self, inputCentroids):   #new: inputCentroids
            # grab the set of object IDs and corresponding inputCentroids
            objectIDs = list(self.objects.keys())
            objectCentroids = list(self.objects.values())


			# compute the distance between each pair of object
			# centroids and input centroids, respectively -- our
			# goal will be to match an input centroid to an existing
			# object centroid
            D = dist.cdist(np.array(objectCentroids), inputCentroids)

			# in order to perform this matching we must (1) find the
			# smallest value in each row and then (2) sort the row
			# indexes based on their minimum values so that the row
			# with the smallest value as at the *front* of the index
			# list
            rows = D.min(axis=1).argsort()
            #rospy.loginfo(rows)
			# next, we perform a similar process on the columns by
			# finding the smallest value in each column and then
			# sorting using the previously computed row index list
            cols = D.argmin(axis=1)[rows]
            #rospy.loginfo(cols)
			# in order to determine if we need to update, register,
			# or deregister an object we need to keep track of which
			# of the rows and column indexes we have already examined

            usedRows = set()

            usedCols = set()

			# loop over the combination of the (row, column) index
			# tuples

            for (row, col) in zip(rows, cols):
				# if we have already examined either the row or
				# column value before, ignore it
				# val

                if row in usedRows or col in usedCols:
                    continue

				# otherwise, grab the object ID for the current row,
				# set its new centroid, and reset the disappeared
				# counter
                objectID = objectIDs[row]
                #rospy.loginfo(self.objects[objectID])
                self.objects[objectID] = inputCentroids[col]
                self.disappeared[objectID] = 0

				# indicate that we have examined each of the row and
				# column indexes, respectively
                usedRows.add(row)
                usedCols.add(col)

			# compute both the row and column index we have NOT yet
			# examined

            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            rospy.loginfo(self.objects[objectID])
            unusedCols = set(range(0, D.shape[1])).difference(usedCols)

			# in the event that the number of object centroids is
			# equal or greater than the number of input centroids
			# we need to check and see if some of these objects have
			# potentially disappeared

            if D.shape[0] >= D.shape[1]:
				# loop over the unused row indexes

                for row in unusedRows:
					# grab the object ID for the corresponding row
					# index and increment the disappeared counter

                    objectID = objectIDs[row]

                    self.disappeared[objectID] += 1

					# check to see if the number of consecutive
					# frames the object has been marked "disappeared"
					# for warrants deregistering the object

                    if self.disappeared[objectID] > self.maxDisappeared:

                        self.deregister(objectID)

			# otherwise, if the number of input centroids is greater
			# than the number of existing object centroids we need to
			# register each new input centroid as a trackable object

            else:

                for col in unusedCols:

                    self.register(inputCentroids[col])

		    # return the set of trackable objects
            return self.objects

    def update(self, rects):
        # check to see if the list of input bounding box rectangles
        # is empty
        if len(rects) == 0:
            # loop over any existing tracked objects and mark them
            # as disappeared
            for objectID in list(self.disappeared.keys()):
                self.disappeared[objectID] += 1
                # if we have reached a maximum number of consecutive
                # frames where a given object has been marked as
                # missing, deregister it
                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)
            # return early as there are no centroids or tracking info
            # to update
            return self.objects

        # initialize an array of input centroids for the current frame
        inputCentroids = np.zeros((len(rects), 2), dtype="int")
        # loop over the bounding box rectangles
        for (i, (x, y)) in enumerate(rects):
            # use the bounding box coordinates to derive the centroid
            cX = int(x)
            cY = int(y)
            inputCentroids[i] = (cX, cY)


        # if we are currently not tracking any objects take the input
        # centroids and register each of them
        if len(self.objects) == 0:
            for i in range(0, len(inputCentroids)):
                self.register(inputCentroids[i])

        # otherwise, are are currently tracking objects so we need to
        # try to match the input centroids to existing object
        # centroids
        else:

            self.connectObj(inputCentroids)

        # return the set of trackable objects
        return self.objects



def main():
    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    required = parser.add_argument_group('required arguments')

    required.add_argument(
        "-c", "--color", dest="color", default="",
        choices=['red', 'green', 'blue', 'yellow'],
        help="The color which is searched for",
        required=True
    )


    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("detect_object_"+'right', anonymous=True)

    node = CentroidTracker('right', args.color)

    rospy.spin()

if __name__ == "__main__":
    main()
