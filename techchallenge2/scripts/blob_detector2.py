#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from racecar_4.techchallenge2.msg import BlobDetections2 as BlobDetections
import math
import time
import sys
import RacecarUtilities


class BlobDetector:
    def __init__(self):
        self.isTesting = False
        self.bridge = CvBridge()
        self.pub_blobs = rospy.Publisher("/blobs", BlobDetections, queue_size=1)
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.processImage, queue_size=1)
        
        if len(sys.argv) == 2:
            def nothing(x):
                pass
            self.isTesting = True
            self.image = np.zeros((720, 1280, 3), np.uint8)
            cv2.namedWindow('HSV')
            cv2.createTrackbar('HL', 'HSV', 0, 180, nothing)
            cv2.createTrackbar('SL', 'HSV', 0, 255, nothing)
            cv2.createTrackbar('VL', 'HSV', 0, 255, nothing)
            cv2.createTrackbar('HU', 'HSV', 0, 180, nothing)
            cv2.createTrackbar('SU', 'HSV', 0, 255, nothing)
            cv2.createTrackbar('VU', 'HSV', 0, 255, nothing)
            self.hl = 0
            self.sl = 0
            self.vl = 0
            self.hu = 0
            self.su = 0
            self.vu = 0
            self.is_tuning = sys.argv[1]
            self.window_thread = RacecarUtilities.StoppableThread(target=self.window_runner)
            self.window_thread.start()
        rospy.loginfo("BlobDetector initialized.")

    def processImage(self, image_msg):
        im = self.bridge.imgmsg_to_cv2(image_msg)
        #im = im[len(im)*.4:]
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        if not self.isTesting:
            self.find_color(im, "red", cv2.bitwise_or(cv2.inRange(hsv, np.array([0, 150, 150]), np.array([10, 255, 255])), cv2.inRange(hsv, np.array([170, 150, 150]), np.array([180, 255, 255]))))  # red
            self.find_color(im, "green", cv2.inRange(hsv, np.array([40, 100, 100]), np.array([85, 255, 255])))  # green
        else:
            self.find_color(im, "testing",cv2.inRange(hsv, np.array([self.hl, self.sl, self.vl]), np.array([self.hu, self.su, self.vu])))

    def find_color(self, passed_im, label_color, mask):
        blob_msg = BlobDetections()
        if self.isTesting:
            self.image = im
        contours = cv2.findContours(mask, cv2.cv.CV_RETR_TREE, cv2.cv.CV_CHAIN_APPROX_SIMPLE)[0]
        approx_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 500: 
                continue
            perim = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, .05*perim, True)
            if len(approx) == 4:
                approx_contours.append(approx)
                moments = cv2.moments(c)
                center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
                cv2.putText(im, label_color, center, cv2.FONT_HERSHEY_PLAIN, 2, (100, 255, 100))
                print "Moment:  ({}, {})".format(center[0], center[1])
                print "Label color:  {}".format(label_color)
                blob_msg.colors.append(label_color)
                blob_msg.heights.append(float((max(approx, key=lambda x: x[0][0][1])[0][1] - min(approx, key=lambda x: x[0][1])[0][1])) / len(im))
                msg_loc = Point(
                msg_loc.x, msg_loc.y = float(center[0]) / len(im[0]), float(center[1]) / len(im)
                blob_msg.locations.append(msg_loc)
                self.pub_blobs.publish(blob_msg)

        if approx_contours:
            if self.isTesting:
                cv2.drawContours(self.image, approx_contours, -1, (100, 255, 100), 2)

       def window_runner(self):
        cv2.imshow('HSV', cv2.resize(self.image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            self.window_thread.stop()
        self.hl = cv2.getTrackbarPos('HL', 'HSV')
        self.sl = cv2.getTrackbarPos('SL', 'HSV')
        self.vl = cv2.getTrackbarPos('VL', 'HSV')
        self.hu = cv2.getTrackbarPos('HU', 'HSV')
        self.su = cv2.getTrackbarPos('SU', 'HSV')
        self.vu = cv2.getTrackbarPos('VU', 'HSV')


if __name__ == "__main__":
    rospy.init_node("BlobDetector")
    bd = BlobDetector()
    rospy.spin()
    cv2.destroyAllWindows()