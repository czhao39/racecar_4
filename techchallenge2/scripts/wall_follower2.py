#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import time, collections


class WallFollower():
    def __init__(self, is_right):
        self.driving = AckermannDriveStamped()
        self.driving.header.stamp = rospy.Time.now()
        self.driving.drive.speed = 10
	if is_right:
            self.ddes = .7
        else:
	    self.ddes = 5
	self.prev_times = collections.deque([time.clock() for _ in range(10)])
        self.prev_errors = collections.deque([0 for _ in range(4)])
        self.kp = .9
        #self.ki = .08
	self.ki = .3
	#self.kd = .05
	self.kd = .02
	if is_right:
            self.mult = 1
            self.start_ind = 80
            self.end_ind = 500
        else:
            self.mult = -1
            self.start_ind = 580
            self.end_ind = 1000
        self.pid_pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.pid_callback)
	print "wall following"

    def pid_callback(self, msg):
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
