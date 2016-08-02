#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import threading
#from racecar_wk3.msg import BlobDetections
from std_msgs.msg import String
#from geometry_msgs.msg import Point
import math
import time
import sys
import RacecarUtilities


class BlobDetector:
    def __init__(self):
        self.isTesting = False
        self.eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')  
        self.bridge = CvBridge()
        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        self.pub_blobs = rospy.Publisher("/exploring_challenge", String, queue_size=1)
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
            self.find_color(im, "yellow", cv2.inRange(hsv, np.array([20, 150, 150]), np.array([30, 255, 255])))  # yellow
            self.find_color(im, "blue", cv2.inRange(hsv, np.array([90, 140, 110]), np.array([130, 255, 255])))  # blue
            self.find_color(im, "pink", cv2.inRange(hsv, np.array([135, 80, 90]), np.array([165, 255, 255])))  # blue
            self.find_faces(im)  # faces
        else:
            self.find_color(im, "testing",cv2.inRange(hsv, np.array([self.hl, self.sl, self.vl]), np.array([self.hu, self.su, self.vu])))

    def find_color(self, passed_im, label_color, mask):
        if self.isTesting:
            self.image = im
        contours = cv2.findContours(mask, cv2.cv.CV_RETR_TREE, cv2.cv.CV_CHAIN_APPROX_SIMPLE)[0]
        for c in contours:
            area = cv2.contourArea(c)
            if area < 500: 
                continue
            perim = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, .03*perim, True)
            if len(approx) == 4:  # rectangle
                rect_msg = String()
                rect_msg.data = "{} rectangle".format(label_color)
                self.pub_blobs.publish(rect_msg)
                moments = cv2.moments(c)
                center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
                im = passed_im.copy()
                cv2.putText(im, "{} rectangle".format(label_color), center, cv2.FONT_HERSHEY_PLAIN, 2, (100, 255, 100))
                cv2.drawContours(im, [approx], -1, (100, 255, 100), 2)
                cv2.imwrite("/home/racecar/challenge_photos1/{}rectangle{}.png".format(label_color, int(time.clock()*1000)), im)

            elif abs(len(approx)-12) <= 2:  # cross
                cross_msg = String()
                cross_msg.data = "{} cross".format(label_color)
                self.pub_blobs.publish(rect_msg)
                moments = cv2.moments(c)
                center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
                im = passed_im.copy()
                cv2.putText(im, "{} cross".format(label_color), center, cv2.FONT_HERSHEY_PLAIN, 2, (100, 255, 100))
                cv2.drawContours(im, [approx], -1, (100, 255, 100), 2)
                cv2.imwrite("/home/racecar/challenge_photos1/{}cross{}.png".format(label_color, int(time.clock()*1000)), im)

            elif len(approx) > 4:
                circ_msg = String()
                circ_msg.data = "{} circle".format(label_color)
                self.pub_blobs.publish(circ_msg)
                moments = cv2.moments(c)
                center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
                im = passed_im.copy()
                cv2.putText(im, "{} circle".format(label_color), center, cv2.FONT_HERSHEY_PLAIN, 2, (100, 255, 100))
                cv2.drawContours(im, [approx], -1, (100, 255, 100), 2)
                cv2.imwrite("/home/racecar/challenge_photos1/{}circle{}.png".format(label_color, int(time.clock()*1000)), im)

            #if self.isTesting:
            #    cv2.drawContours(self.image, approx_contours, -1, (100, 255, 100), 2)
            #else:
            #    cv2.drawContours(im, approx_contours, -1, (100, 255, 100), 2)

    def find_faces(self, passed_im):
        im = passed_im.copy()
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        found_face = False
        for (x, y, w, h) in faces:
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = im[y:y+h, x:x+w]
            eyes = self.eye_cascade.detectMultiScale(roi_gray)
            if len(eyes) == 0: continue
            found_face = True
            cv2.rectangle(im, (x, y), (x+w, y+h), (100, 255, 100), 2)
            eh = eyes[0][3]
            if eh == 0: continue
            print "face height: {},  eye height: {}".format(h, eh)
            if abs(h/eh - 1.5) < 1.5:
                print "detected karaman"
                cv2.imwrite("/home/racecar/challenge_photos/karaman{}.png".format(int(time.clock()*1000)), im)
                blob_msg = String()
                blob_msg.data = "professor karaman"
                self.pub_blobs.publish(blob_msg)
            elif abs(h/eh - 6) < 3:
                print "detected ari"
                cv2.imwrite("/home/racecar/challenge_photos/ari{}.png".format(int(time.clock()*1000)), im)
                blob_msg = String()
                blob_msg.data = "ari"
                self.pub_blobs.publish(blob_msg)
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
