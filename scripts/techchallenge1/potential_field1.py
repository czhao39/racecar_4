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

# Import numpy for sanity
from rospy.numpy_msg import numpy_msg
import numpy as np
import math

# simple class to contain the node's variables and code
class PotentialField:
    # class constructor; subscribe to topics and advertise intent to publish
    def __init__(self):
        # initialize potential field variables
        self.charge_laser_particle = 0.07
        #self.charge_laser_particle = 0.04
        self.charge_forward_boost = 25.0
        self.boost_distance = 0.5
        self.p_speed = 0.007
        self.p_steering = 1.0

        # subscribe to laserscans. Force output message data to be in numpy arrays.
        rospy.Subscriber("/scan", numpy_msg(LaserScan), self.scan_callback)

        # output a pose of where we want to go
        self.pub_goal = rospy.Publisher("~potentialFieldGoal", PointStamped, queue_size=1)
        self.pub_nav = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)

    def scan_callback(self, msg):
        # Debug
        #print("Starting increment {} increment {}".format(msg.angle_min, msg.angle_increment))

        # Create potential gradients for all laser scan particles
        scan_rad_angles = ( (msg.angle_increment * np.arange(1081, dtype=float)) + msg.angle_min )

        scan_x_unit_vectors = -np.cos(scan_rad_angles)
        scan_y_unit_vectors = -np.sin(scan_rad_angles)

        scan_x_components = (self.charge_laser_particle * scan_x_unit_vectors) / np.square(msg.ranges)
        scan_y_components = (self.charge_laser_particle * scan_y_unit_vectors) / np.square(msg.ranges)

        # Add the potential for the point behind the robot (to give it a kick)
        kick_x_component = np.ones(1) * self.charge_forward_boost / self.boost_distance**2.0
        kick_y_component = np.zeros(1)

        # Add together the gradients to create a global gradient showing the robot which direction to travel in
        total_x_component = np.sum(scan_x_components) + kick_x_component
        total_y_component = np.sum(scan_y_components) + kick_y_component

        # Transform this gradient vector into a PoseStamped object
        visualizer_msg = PointStamped()
        visualizer_msg.header.frame_id = 'base_link'
        visualizer_msg.point.x = total_x_component
        visualizer_msg.point.y = total_y_component

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
