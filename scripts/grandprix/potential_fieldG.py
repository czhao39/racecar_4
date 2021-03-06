#!/usr/bin/env python

'''
Created by Winter Guerra <winterg@mit.edu> on July 2016.
'''

# import main ROS python library
import rospy

# import message types
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String

# Import numpy for sanity
from rospy.numpy_msg import numpy_msg
import numpy as np
import math
import sys
from racecar_4.msg import BlobDetectionsG as BlobDetections

# simple class to contain the node's variables and code
class PotentialField:
    # class constructor; subscribe to topics and advertise intent to publish
    def __init__(self):
        # initialize potential field variables
        self.charge_laser_particle = 0.07 * 2
        #self.charge_laser_particle = 0.04
        self.charge_forward_boost = 25.0 * 2
        self.boost_distance = 0.5
        self.p_speed = 0.007
        self.p_steering = 1.0
        self.isTesting = False
        self.turn_vect = 0
        self.blob_sub = rospy.Subscriber("/blobs", BlobDetections, self.set_turn_vect, queue_size=1)
        self.turn_start = 0
        self.turn_count = 0
        self.shortcut_detect = False  ####################

        # subscribe to laserscans. Force output message data to be in numpy arrays.
        rospy.Subscriber("/scan", numpy_msg(LaserScan), self.scan_callback)

        # output a pose of where we want to go
        self.pub_goal = rospy.Publisher("~potentialFieldGoal", PointStamped, queue_size=1)
        self.pub_nav = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)

        if len(sys.argv) == 2:
            self.isTesting = True

    def set_turn_vect(self, msg):
        if len(msg.heights) == 0: return
        closest_ind = max(enumerate(msg.areas), key=lambda x: x[1])[0]
        if msg.heights[closest_ind] > .2:
            if msg.colors[closest_ind] == "blue":
                self.shortcut_detect = True
                rospy.loginfo("detected blue")
            elif self.shortcut_detect:
                if msg.colors[closest_ind] == "red":
                    self.turn_start = rospy.get_time()
                    self.turn_count += 1
                    if self.turn_count > 3:
                        self.turn_vect = -300
                        rospy.loginfo("avoiding shortcut")
                elif msg.colors[closest_ind] == "green":
                    self.turn_start = rospy.get_time()
                    self.turn_count += 1
                    if self.turn_count > 5:
                        self.turn_vect = 20
                        rospy.loginfo("entering shortcut")

    def scan_callback(self, msg):
        # Debug
        #print("Starting increment {} increment {}".format(msg.angle_min, msg.angle_increment))

        vectors = []

        # Create potential gradients for all laser scan particles
        scan_rad_angles = ( (msg.angle_increment * np.arange(1081, dtype=float)) + msg.angle_min )

        scan_x_unit_vectors = -np.cos(scan_rad_angles)
        scan_y_unit_vectors = -np.sin(scan_rad_angles)

        ranges = np.array(msg.ranges) - .1
        scan_x_components = (self.charge_laser_particle * scan_x_unit_vectors) / np.square(ranges)
        scan_y_components = (self.charge_laser_particle * scan_y_unit_vectors) / np.square(ranges)
        
        # Add the potential for the point behind the robot (to give it a kick)
        kick_x_component = np.ones(1) * self.charge_forward_boost / self.boost_distance**2.0
        kick_y_component = np.zeros(1)

        # Vector to farthest point in front of car
        farthest_ind = max((i for i in range(180, 901, 4)), key=lambda i: sum(msg.ranges[i:i+4])/4)
        #dist = sum(msg.ranges[farthest_ind:farthest_ind+4]) / 4
        far_x_component = math.cos(math.radians(farthest_ind/4-135)) * 60
        far_y_component = math.sin(math.radians(farthest_ind/4-135)) * 60
        
        if self.turn_count != 0 and rospy.get_time() - self.turn_start > 1:
            self.turn_count = 0
            self.turn_vect = 0
            self.blob_sub.unregister()
        
        #rospy.loginfo("far_vect_x:  {}, far_vect_y:  {}".format(far_x_component, far_y_component))
        
        # Add together the gradients to create a global gradient showing the robot which direction to travel in
        total_x_component = np.sum(scan_x_components) + kick_x_component + far_x_component
        total_y_component = (np.sum(scan_y_components) + kick_y_component + self.turn_vect + far_y_component) / 3
        rospy.loginfo("x comp:  {}, y comp:  {}i\n".format(total_x_component, total_y_component))

        # Transform this gradient vector into a PoseStamped object
        visualizer_msg = PointStamped()
        visualizer_msg.header.frame_id = 'base_link'
        visualizer_msg.point.x = total_x_component
        visualizer_msg.point.y = total_y_component

        if self.isTesting:
            vectors.append(("LaserScan",int(np.sum(scan_x_components)), int(np.sum(scan_y_components))))
            vectors.append(("Kick",int(kick_x_component), int(kick_y_component)))
            vectors.append(("Push",int(far_x_component), int(far_y_component)))
            vectors.append(("Total",int(total_x_component), int(total_y_component)))
            # Convert the vectors to a String and publish them
            vecString = ""
            for vector in vectors:
                for i, item in enumerate(vector):
                    if (i == len(vector)-1): vecString+=(str(item))
                    else: vecString+=(str(item)+",")
                vecString+="\n"
            f = open('/home/racecar-4/racecar-ws/src/racecar/racecar_4/scripts/grandprix/vectorData.txt','w')
            f.write(vecString)
            f.close()

        # Publish this goal so that we can see it in RVIZ
        self.pub_goal.publish(visualizer_msg)

        # Now, create a steering command to send to the vesc.
        command_msg = AckermannDriveStamped()
        command_msg.drive.steering_angle = (self.p_steering * np.sign(total_x_component) * math.atan2(total_y_component, total_x_component))
        
        # r = rospy.Rate(3)
        # for i in range(3):
        
        command_msg.drive.speed = (self.p_speed * np.sign(total_x_component) * math.sqrt(total_x_component**2 + total_y_component**2))

        # Publish the command
        self.pub_nav.publish(command_msg)



if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    rospy.init_node("potential_field_node")

    node = PotentialField()

    # enter the ROS main loop
    rospy.spin()
