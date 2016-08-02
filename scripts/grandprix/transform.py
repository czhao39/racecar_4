#!/usr/bin/env python

import rospy
from tf import TransformListener
from tf2_msgs.msg import TFMessage


class TransformNode:
    def __init__(self, *args):
        self.tf = TransformListener()
        rospy.Subscriber("/tf", TFMessage, transform, queue_size=1)

    def transform(self, msg):
        if self.tf.frameExists("/base_link") and self.tf.frameExists("/map"):
            t = self.tf.getLatestCommonTime("/base_link", "/map")
            position, quaternion = self.tf.lookupTransform("/base_link", "/map", t)
            print position, quaternion
