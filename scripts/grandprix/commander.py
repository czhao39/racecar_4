#!/usr/bin/env python
"""
commander(git).py

MIT RACECAR 2016

This class will act as a commanding node that reads
(processed) message data from accompanying nodes, makes
decisions on how to proceed, then publishes drive commands
directly to the /navigation topic.

"""


# IMPORTS

import rospy
import math
import time

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

from racecar_wk3.msg import BlobDetections
from racecar_wk3.msg import ObjectDetections


# CLASS DECLARATION

class Commander:

    def __init__(self):
        self.DrivePub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,queue_size=0)
        # Add any other topic variables here

        self.SPEED = 2
        # Add any other class constants here

        self.prev_error = 0
        # Add any other class variables here
   
        self.WALL_KP = .9
        self.WALL_KI = .3
        self.WALL_KD = .02
        self.wall_prev_error = 0
        self.wall_prev_time = time.clock()
        self.WALL_DDES = 0.4
        self.wall_right = False  # which wall to follow

        self.OBJ_KP = .01
        self.OBJ_KI = .00
        self.OBJ_KD = 0
        self.obj_prev_error = 0
        self.obj_prev_time = time.clock()


    # Function: drive
    # Parameters: speed (float), angle (float)
    #
    # This function will publish a drive command to
    # the /navigation topic in ROS

    def drive(self, angle):
        msg = AckermannDriveStamped()           # Initializes msg variable
        msg.drive.speed = self.SPEED            # Sets msg speed to entered speed
        msg.drive.acceleration = 0              # Sets msg acceleration to 0
        msg.drive.jerk = 1                      # Sets msg jerk to 1
        msg.drive.steering_angle = angle        # Sets msg steering angle to entered angle
        msg.drive.steering_angle_velocity = 1   # Sets msg angle velocity to 1
        self.DrivePub.publish(msg)              # Publishes the message

    def evade_objects(self, msg):
        #msg.dists += (.01,)  # fake left wall
        #msg.lefts += (140,)
        #msg.rights += (270,)
        objs = [d for d in msg.dists if d < 3]  # dists in order of distance
        objs.sort()
	if len(objs) >= 2:
            thresh_dist = objs[1]
        elif objs:
            thresh_dist = objs[len(objs)-1]
        else:
            self.drive(0)
            return
        close_objs = [i for i in range(len(msg.dists)) if msg.dists[i]-thresh_dist < .2]  # consider objects that are 1 meter within closest object
        max_space = -1
        max_center = None
        for s in range(len(close_objs)-1):
            space = msg.lefts[close_objs[s+1]] - msg.rights[close_objs[s]]
            if space > max_space:
                max_space = space
                max_center = (msg.lefts[close_objs[s+1]] + msg.rights[close_objs[s]]) / 2
        if max_center is None:
            rospy.loginfo("no space, we're doomed")
        else:
            error = max_center - 135
            if abs(error) > 3: 
                # PUBLISH DRIVE COMMAND
                self.drive(self.calc_pid(self.OBJ_KP, self.OBJ_KD, self.OBJ_KI, error, self.obj_prev_error, self.obj_prev_time))    # Execute drive function
            else:
                self.drive(0)
            self.obj_prev_error = error
        self.obj_prev_time = time.clock()


    # Function: wall_follow
    # Parameters: msg (ObjectDetection)
    #
    # This function uses a PID control system
    # to let the car follow a line

    def wall_follow(self, msg):

        if self.wall_right:
            start_angle = 20
            end_angle = 125
            mult = 1
        else:
            start_angle = 145
            end_angle = 250
            mult = -1
        
        try:
            min_ind = min([i for i in range(len(msg.dists)) if (start_angle < msg.lefts[i] and msg.lefts[i] < end_angle) or (start_angle < msg.rights[i] and msg.rights[i] < end_angle) or (msg.lefts[i] < start_angle and msg.rights[i] > end_angle)], key=lambda x: msg.dists[x])
        except ValueError:  # no wall detected
            rospy.loginfo("no wall detected")
            self.drive(0)
            return

        dist = msg.dists[min_ind]       # Finds the minimum range

        error = self.WALL_DDES - dist
        
        # SET PID PARAMETERS
        THRESHOLD = 0.05                # Sets threshold to 5cm
        
        if abs(error) > THRESHOLD: 
            # PUBLISH DRIVE COMMAND
            self.drive(mult * self.calc_pid(self.WALL_KP, self.WALL_KD, self.WALL_KI, error, self.wall_prev_error, self.wall_prev_time))    # Execute drive function
        else:
            self.drive(0)
        
        self.wall_prev_error = error
        self.wall_prev_time = time.clock()

    def calc_pid(self, KP, KD, KI, error, prev_error, prev_time):
        e_deriv = (error - prev_error) / (time.clock() - prev_time)
        e_int = (error + prev_error) / 2 * (time.clock() - prev_time)
        return KP*error + KD*e_deriv + KI*e_int
