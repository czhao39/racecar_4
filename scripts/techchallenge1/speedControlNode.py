#!/usr/bin/env python


"""
commanderNode.py

MIT RACECAR 2016

This program implements the commander class and manages
the advanced computations required by the RACECAR

"""

# IMPORTS

import rospy
import math

from ackermann_msgs.msg import AckermannDriveStamped



# VARIABLES

SUBSCRIBE_TO = "/drive"
PUBLISH_TO = '/vesc/ackermann_cmd_mux/input/navigation'

NODE_NAME = 'speedController'

prev_speed = 0
THRESHOLD = 0.1     # threshold for acceleration

# CALLBACK

def callBack(msg):
    global prev_speed

    # if speed is increasing
    if (msg.drive.speed > prev_speed):  

        # if accelerating too quickly
        if (abs(msg.drive.speed - prev_speed) > THRESHOLD):

            msg.drive.speed = prev_speed + THRESHOLD

    # updates the previous speed
    prev_speed = msg.drive.speed

    # publishes the new drive command
    drivePub = rospy.Publisher(PUBLISH_TO, AckermannDriveStamped,queue_size=10)
    drivePub.publish(msg)  # Publishes the message


# MAIN()      

rospy.init_node(NODE_NAME)

msg = rospy.Subscriber(SUBSCRIBE_TO, AckermannDriveStamped, callBack)

rospy.spin()
