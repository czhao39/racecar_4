#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from racecar_wk3.msg import ObjectDetections


class SafetyController:
    def __init__(self):
        self.safety_pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=0)
        self.objects_sub = rospy.Subscriber("objects", ObjectDetections, self.objects_cb)

    def objects_cb(self, msg):
        self.objects_sub.unregister()
        for o in range(len(msg.dists)):
            if msg.dists[o] < .5 and ((120 < msg.lefts[o] and msg.lefts[o] < 150) or (120 < msg.rights[o] and msg.rights[o] < 150) or (msg.lefts[o] < 120 and msg.rights[o] > 150)):
                stop = AckermannDriveStamped()
                stop.header.stamp = rospy.Time.now()
                stop.drive.speed = -1
                r = rospy.Rate(6)
                for i in range(3):
                    self.safety_pub.publish(stop)
                    r.sleep()
                break
        self.objects_sub = rospy.Subscriber("objects", ObjectDetections, self.objects_cb)


if __name__ == "__main__":
    rospy.init_node("safety")
    sc = SafetyController()
    rospy.spin()
