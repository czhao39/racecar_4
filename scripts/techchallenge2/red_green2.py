#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
from racecar_4.msg import BlobDetections2 as BlobDetections
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from wall_follower2 import WallFollower


class RedGreen:
    def __init__(self):
        self.move = AckermannDriveStamped()
        self.move.drive.speed = 3
        self.move.header.stamp = rospy.Time.now()
	self.pub_move = rospy.Publisher("vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        self.sub_blob = rospy.Subscriber("/blobs", BlobDetections, self.cbBlobs, queue_size=1)
        
        self.kp = .3
        #self.ki = .05
	self.ki = 0
	#self.ki = .03
	#self.kd = .06
        self.kd = 0
	self.prev_error = 0
        self.prev_time = time.clock()
        
        rospy.loginfo("RedGreen initialized.")

    def cbBlobs(self, msg):
	rospy.loginfo("Got blobs message.")
        if not msg.sizes:  # no blobs
            self.move.drive.speed = 3
	    self.move.drive.steering_angle = 0
        else:
            closest_ind = max(enumerate(msg.heights), key=lambda x: x[1].data)[0]
            rospy.loginfo("Blob size:  {}".format(msg.heights[closest_ind].data))
	    if msg.heights[closest_ind].data > .12:
                if (msg.colors[closest_ind] == "red":  # red
                    self.right = False
                else:
                    self.right = True
                self.sub_blob.unregister()
		self.move.drive.speed = 0
		self.pub_move.publish(self.move)
                self.move.drive.speed = -.5
		self.move.drive.steering_angle = 0
		self.move_back_sub = rospy.Subscriber("scan", LaserScan, self.move_back_cb)

            else:
		self.move.drive.speed = 2
                error = .5 - msg.locations[closest_ind].x
		rospy.loginfo("Error:  {}".format(error))
                if abs(error) > .01:
                    self.move.drive.steering_angle = self.calc_pid(error)
		else:
		    self.move.drive.steering_angle = 0
	self.move.header.stamp = rospy.Time.now()
	self.pub_move.publish(self.move)
	rospy.loginfo("Sent move msg")

    def calc_pid(self, error):
        e_deriv = (error - self.prev_error) / (time.clock() - self.prev_time)
        e_int = (error + self.prev_error)/2 * (time.clock() - self.prev_time)
        self.prev_error = error
        self.prev_time = time.clock()
        return self.kp*error + self.kd*e_deriv + self.ki*e_int

    def turn_back_cb(self, msg):
        averaged = [sum(msg.ranges[i:i+4])/4 for i in range(0, 1080, 4)]
        closest = min(enumerate(averaged), key=lambda x: x[1])[0]
	print "turning back"
	if abs(135 - closest) < 6:  # within 10 deg
            print "perpendicular"
	    self.move.drive.steering_angle = 0
            self.turn_back_sub.unregister()
            self.move_back_sub = rospy.Subscriber("scan", LaserScan, self.move_back_cb)
        elif closest > 135:
	    self.move.drive.steering_angle = -.3
	else:
	    self.move.drive.steering_angle = .3
	self.pub_move.publish(self.move)

    def move_back_cb(self, msg):
        if self.calcDistance(msg.ranges, "F") > .7:
            self.move_back_sub.unregister()
	    self.move.drive.speed = 0
	    self.pub_move.publish(self.move)
            self.move.drive.speed = 2
	    self.quarter_turn()

        self.pub_move.publish(self.move)

    def calcDistance(self, ranges, side):
    	lengthx = 0
	lengthy = 0
	total = 0
	if side == "F":
	    total = 0
	    for count in range(530, 550):
		total += ranges[count]
	    return total/20

    def quarter_turn(self):
        print "started quarter turn"
        self.move.drive.speed = 1
	if self.right:
	    self.move.drive.steering_angle = -10
	    count = 7
	else:
	    self.move.drive.steering_angle = 10
	    count = 13
	r = rospy.Rate(6)
	for i in range(count):
	    self.pub_move.publish(self.move)
	    r.sleep()
	wall_controller = WallFollower(not self.right)  # start wall following
	

if __name__ == "__main__":
    rospy.init_node("RedGreen")
    pc = RedGreen()
    rospy.spin()
