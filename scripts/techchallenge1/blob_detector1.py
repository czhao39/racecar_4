#!/usr/bin/env python

import numpy as np
from skimage.measure import structural_similarity as ssim
import cv2
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#from racecar_wk3.msg import BlobDetections
from std_msgs.msg import String
#from geometry_msgs.msg import Point

import math
import time
import sys
import RacecarUtilities1 as RacecarUtilities

SERTAC_IMG = cv2.resize(cv2.imread('/home/racecar/racecar-ws/src/racecar/racecar_4/scripts/techchallenge1/sertac.png',0), (100,100), interpolation = cv2.INTER_AREA)
ARI_IMG = cv2.resize(cv2.imread('/home/racecar/racecar-ws/src/racecar/racecar_4/scripts/techchallenge1/ari.png',0), (100,100), interpolation = cv2.INTER_AREA)
CAT_IMG = cv2.resize(cv2.imread('/home/racecar/racecar-ws/src/racecar/racecar_4/scripts/techchallenge1/cat.png',0), (100,100), interpolation = cv2.INTER_AREA)
RACECAR_IMG = cv2.resize(cv2.imread('/home/racecar/racecar-ws/src/racecar/racecar_4/scripts/techchallenge1/racecar.png',0), (100,100), interpolation = cv2.INTER_AREA)

class BlobDetector:
    def __init__(self):
        self.isTesting = False
        self.bridge = CvBridge()
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
    def classify(self, passed_im):
        """
        def display_compare_images(imageA, imageB, title):
            # compute the structural similarity
            # index for the images
            s = ssim(imageA, imageB)
        
            # setup the figure
            fig = plt.figure(title)
            plt.suptitle("SSIM: %.2f" % s)
        
            # show first image
            ax = fig.add_subplot(1, 2, 1)
            plt.imshow(imageA, cmap = plt.cm.gray)
            plt.axis("off")
        
            # show the second image
            ax = fig.add_subplot(1, 2, 2)
            plt.imshow(imageB, cmap = plt.cm.gray)
            plt.axis("off")
        
            # show the images
            plt.show()
        """
        classification = "None"
        img1 = cv2.resize(cv2.cvtColor(passed_im, cv2.COLOR_BGR2GRAY), (100, 100), interpolation = cv2.INTER_AREA) 
        img2 = SERTAC_IMG

        imageClasses = SERTAC_IMG, ARI_IMG, CAT_IMG, RACECAR_IMG
        similarities = []
        classifications = ["sertac", "ari", "cat", "racecar"]
        for image in imageClasses:
            similarities.append(ssim(image, img1))
            # display compare the images
            #display_compare_images(image, img1, "Source vs. Compare")
        if similarities:
            classification = classifications[similarities.index(max(similarities))]

        return classification

    def processImage(self, image_msg):
        im = self.bridge.imgmsg_to_cv2(image_msg)
        #im = im[len(im)*.4:]
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        if not self.isTesting:
            self.find_color(im, "red", cv2.inRange(hsv, np.array([0, 130, 50]), np.array([10, 255, 140])))     # red
            self.find_color(im, "green", cv2.inRange(hsv, np.array([40, 100, 40]), np.array([85, 255, 200])))  # green
            self.find_color(im, "yellow", cv2.inRange(hsv, np.array([20, 165, 80]), np.array([40, 200, 120])))  # yellow
            self.find_color(im, "blue", cv2.inRange(hsv, np.array([100, 115, 55]), np.array([130, 255, 255])))  # blue
            self.challenge_color(im, "pink", cv2.inRange(hsv, np.array([120, 80, 65]), np.array([180, 185, 170])))  # pink
        else:
        	pass
            #self.find_color(im, "testing",cv2.inRange(hsv, np.array([self.hl, self.sl, self.vl]), np.array([self.hu, self.su, self.vu])))
            #self.challenge_color(im, "challenge testing", cv2.inRange(hsv, np.array([self.hl, self.sl, self.vl]), np.array([self.hu, self.su, self.vu])))  # pink
    def find_color(self, passed_im, label_color, mask):
    	im = passed_im.copy()
        if self.isTesting:
            self.image = im
        contours = cv2.findContours(mask, cv2.cv.CV_RETR_TREE, cv2.cv.CV_CHAIN_APPROX_SIMPLE)[0]
        approx_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 500: 
                continue
            perim = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, .02*perim, True)
            if len(approx) == 4:  # rectangle
            	approx_contours.append(approx)
                rect_msg = String()
                rect_msg.data = "{} rectangle".format(label_color)
                self.pub_blobs.publish(rect_msg)
                moments = cv2.moments(c)
                center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
                im = passed_im.copy()
                cv2.putText(im, "{} rectangle".format(label_color), center, cv2.FONT_HERSHEY_PLAIN, 2, (100, 255, 100))
                cv2.drawContours(im, [approx], -1, (100, 255, 100), 2)
                cv2.imwrite("/home/racecar/challenge_photos1/{}rectangle{}.png".format(label_color, int(time.clock()*1000)), im)
                print "{}square".format(label_color)
            elif abs(len(approx)-12) <= 1:  # cross
            	approx_contours.append(approx)
                cross_msg = String()
                cross_msg.data = "{} cross".format(label_color)
                self.pub_blobs.publish(cross_msg)
                moments = cv2.moments(c)
                center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
                im = passed_im.copy()
                cv2.putText(im, "{} cross".format(label_color), center, cv2.FONT_HERSHEY_PLAIN, 2, (100, 255, 100))
                cv2.drawContours(im, [approx], -1, (100, 255, 100), 2)
                cv2.imwrite("/home/racecar/challenge_photos1/{}cross{}.png".format(label_color, int(time.clock()*1000)), im)
                print "{}cross".format(label_color)
            elif abs(len(approx)-8) <= 2:  # circle

            	approx_contours.append(approx)
                circ_msg = String()
                circ_msg.data = "{} circle".format(label_color)
                self.pub_blobs.publish(circ_msg)
                moments = cv2.moments(c)
                center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
                im = passed_im.copy()
                cv2.putText(im, "{} circle".format(label_color), center, cv2.FONT_HERSHEY_PLAIN, 2, (100, 255, 100))
                cv2.drawContours(im, [approx], -1, (100, 255, 100), 2)
                cv2.imwrite("/home/racecar/challenge_photos1/{}circle{}.png".format(label_color, int(time.clock()*1000)), im)
                print "{}circle".format(label_color)
            if self.isTesting:
                cv2.drawContours(self.image, approx_contours, -1, (100, 255, 100), 2)
            #else:
            #    cv2.drawContours(im, approx_contours, -1, (100, 255, 100), 2)
    def challenge_color(self, passed_im, label_color, mask):
        im = passed_im.copy()
        if self.isTesting:
            self.image = im
        contours = cv2.findContours(mask, cv2.cv.CV_RETR_TREE, cv2.cv.CV_CHAIN_APPROX_SIMPLE)[0]
        crop_img = None
        approx_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 500: 
                continue
            perim = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, .03*perim, True)
            if len(approx) == 4:  # rectangle
            	approx_contours.append(approx)
                flattenPoints = approx.reshape(4, 2) #Reshape retval of approxPolyDP to just np.ndarry() of points
                
                #Routine to find TopLeft Corner of a list of points returned by approxPolyDP            
                topLeftPoints = flattenPoints.copy()  #Copy over flatten points
                xTopRemove = topLeftPoints.T[0].argsort()[-2:] #Find the indices of the two points with the largest x values
                for index in sorted(xTopRemove, reverse=True): #Remove those points
                    topLeftPoints = np.delete(topLeftPoints, index, axis=0)
                yTopRemove = topLeftPoints.T[1].argsort()[:1]  #Find the index of the point with the largest y value
                for index in sorted(yTopRemove, reverse=True): #Remove that points
                    topLeftPoints = np.delete(topLeftPoints, index, axis=0)
                topLeft = topLeftPoints.reshape(2)

                #Routine to find BottomRight Corner of a list of points returned by approxPolyDP            
                bottomRightPoints = flattenPoints.copy()  #Copy over flatten points
                xBottomRemove = bottomRightPoints.T[1].argsort()[-2:] #Find the indices of the two points with the largest y values
                for index in sorted(xBottomRemove, reverse=True): #Remove those points
                    bottomRightPoints = np.delete(bottomRightPoints, index, axis=0)
                yBottomRemove = bottomRightPoints.T[0].argsort()[:1]  #Find the index of the point with the smallest x value
                for index in sorted(yBottomRemove, reverse=True): #Remove that points
                    bottomRightPoints = np.delete(bottomRightPoints, index, axis=0)
                bottomRight = bottomRightPoints.reshape(2)

                x1 = topLeft[0]
                y2 = topLeft[1]
                x2 = bottomRight[0]
                y1 = bottomRight[1]
                crop_img = passed_im[y1:y2, x1:x2] # Crop from x, y, w, h -> 100, 200, 300, 400
                classification = self.classify(crop_img)

                rect_msg = String()
                rect_msg.data = "{} {}".format(classification, label_color)
                self.pub_blobs.publish(rect_msg)
                moments = cv2.moments(c)
                center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
                im = passed_im.copy()
                cv2.putText(im, "{} {}".format(classification, label_color), center, cv2.FONT_HERSHEY_PLAIN, 2, (100, 255, 100))
                cv2.drawContours(im, [approx], -1, (100, 255, 100), 2)
                cv2.imwrite("/home/racecar/challenge_photos/{}{}{}.png".format(label_color, classification, int(time.clock()*1000)), im)
                print classification
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
