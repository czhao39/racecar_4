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

from commander import Commander

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

from racecar_wk3.msg import BlobDetections
from racecar_wk3.msg import ObjectDetections



# VARIABLES

OBJECT_TOPIC = "/objects"
#BLOB_TOPIC = "/blobs"
#MAP_TOPIC = "/map"

NODE_NAME = 'commander'

#D_DESIRED = 0.8

cody = Commander()

def callback(msg):
    cody.evade_objects(msg)

# MAIN()      

rospy.init_node(NODE_NAME)

object_sub = rospy.Subscriber(OBJECT_TOPIC, ObjectDetections, callback)
#blob_msg = rospy.Subscriber(BLOB_TOPIC, BlobDetections, callBack)
#map_msg = rospy.Subscriber(MAP_TOPIC, MapDetections, callBack)

#sideEntry = rospy.Subscriber("", ,)

rospy.spin()
