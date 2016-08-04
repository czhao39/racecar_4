#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import time, collections
from std_msgs.msg import String


class WallFollower():
    def __init__(self):
        self.driving = AckermannDriveStamped()
        self.driving.header.stamp = rospy.Time.now()
        self.driving.drive.speed = 10
	self.ddes = .8
	self.prev_times = collections.deque([time.clock() for _ in range(10)])
        self.prev_errors = collections.deque([0 for _ in range(4)])
        self.kp = .9
	self.ki = .3
	self.kd = .02
        self.blob_sub = rospy.Subscriber("/blobs", BlobDetections, self.blob_callback)
        self.mult = 1  # right
        self.start_ind = 80
        self.end_ind = 500
        self.turn_start = 0
        self.pid_pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.pid_callback)
	print "wall following"

    def blob_callback(self, msg):
        if len(msg.heights) == 0: return
        closest_ind = max(enumerate(msg.heights), key=lambda x: x[1])[0]
        if msg.heights[closest_ind] > .06:
            if msg.colors[closest_ind] == "red":
                self.mult = 1
                self.start_ind = 80
                self.end_ind = 500
                self.ddes = .4
                self.turn_start = rospy.get_time()
                rospy.loginfo("avoiding shortcut")
            elif msg.colors[closest_ind] == "green":
                self.mult = -1
                self.start_ind = 580
                self.end_ind = 1000
                self.ddes = .4
                self.turn_start = rospy.get_time()
                rospy.loginfo("entering shortcut")

    def pid_callback(self, msg):
        if self.ddes != .8 and rospy.get_time() - self.turn_start > 1:
            self.mult = 1
            self.start_ind = 80
            self.end_ind = 500
            self.ddes = .8
        side = msg.ranges[self.start_ind:self.end_ind]
        dist = sum(side) / len(side)
        error = self.ddes - dist
        if abs(error) < .02:
            self.driving.drive.steering_angle = 0
        else:
            self.driving.drive.steering_angle = self.mult * self.pid(self.kp, self.kd, self.ki, error)
        self.pid_pub.publish(self.driving)

    def pid(self, kp, kd, ki, error):
        prev_error = self.prev_errors.popleft()
        prev_time = self.prev_times.popleft()
        e_deriv = (error - prev_error) / (time.clock() - prev_time)
        e_int = (error + prev_error) / 2 * (time.clock() - prev_time)
        self.prev_times.append(time.clock())
        self.prev_errors.append(error)
        return kp * error + kd * e_deriv + ki * e_int


if __name__ == "__main__":
    rospy.init_node("WallFollower")
    controller = WallFollower()
    rospy.spin()
