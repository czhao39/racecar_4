#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped


class StopStopper:
    def __init__(self):
        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=1)
        self.sub = rospy.Subscriber("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, self.stop_stopping, queue_size=1)
        #self.sub = rospy.Subscriber("/drive", AckermannDriveStamped, self.stop_stopping, queue_size=1)
        self.stop_start = rospy.get_time()

    def stop_stopping(self, msg):
        if abs(msg.drive.speed) < .2:
            if rospy.get_time() - self.stop_start > 4:
                self.stop_start = rospy.get_time()
                msg = AckermannDriveStamped()
                msg.header.stamp = rospy.Time.now()
                msg.drive.speed = -1
                msg.drive.steering_angle = -1
                r = rospy.Rate(5)
                print "stop stopping..."
                for _ in range(3):
                    self.pub.publish(msg)
                    r.sleep()
                msg.drive.speed = 1
                msg.drive.steering_angle = 1
                for _ in range(3):
                    self.pub.publish(msg)
                    r.sleep()
        else:
            self.stop_start = rospy.get_time()


if __name__ == "__main__":
    rospy.init_node("stop_stopper")
    ss = StopStopper()
    rospy.spin()
