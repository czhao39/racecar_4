#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from racecar_wk3.msg import ObjectDetections
import math


class ObjectDetector:
    def __init__(self):
        self.object_pub = rospy.Publisher("objects", ObjectDetections, queue_size=0)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_cb)
    
    def scan_cb(self, msg):
        lefts = []
        rights = []
        dists = []
        running_sum = count_object = count_far = 0
        min_object = 100000  # closest point of object
        for i, r in enumerate(msg.ranges):
            if r < 5 and (not count_object or abs(r - running_sum/count_object) < 0.1):  # if same object or new object
                running_sum += r
                count_object += 1
                count_far = 0
                if r < min_object: min_object = r
            else:
                count_far += 1
                if count_far == 15 and count_object:
                    #avg_r = running_sum / count_object
                    #width = count_close / 1080 * 3/2 * math.pi * avg_r  # arc length
                    #if width > .005:  # if width of object larger than 1 cm
                    if count_object > 15:  # detected an actual object
			lefts.append((i-count_far-count_object) / 4)
                        rights.append((i-count_far) / 4)
                        dists.append(min_object)
                    running_sum = 0
                    count_object = 0
                    min_object = 100000
        if count_object:  # check last window
            #width = count_close / 1080 * 3/2 * math.pi * avg_r  # arc length
            #if width > .005:  # if width of object larger than 1 cm
            if count_object > 15:
                lefts.append((1080-count_far-count_object) / 4)
                rights.append(1080 / 4)
                dists.append(min_object)
        if dists:  # only publish if objects detected
            detections = ObjectDetections()
            detections.lefts = lefts  # in degrees
            detections.rights = rights  # in degrees
            detections.dists = dists
	    print "Lefts: {}".format(lefts)
	    print "Rights: {}".format(rights)
	    print "Dists: {}\n".format(dists)
            self.object_pub.publish(detections)


if __name__ == "__main__":
    rospy.init_node("object_detector")
    od = ObjectDetector()
    rospy.spin()
