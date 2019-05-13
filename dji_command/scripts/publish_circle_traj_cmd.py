#!/usr/bin/python
# -*- coding: utf-8 -*-

# Start up ROS pieces.
PKG = 'dji_command'
import roslib; roslib.load_manifest(PKG)
import rospy
import std_msgs
import math

# ROS messages.
from trajectory_msgs.msg import MultiDOFJointTrajectory

class TrajSender():
    def __init__(self):
        pub_traj = rospy.Publisher("command/trajectory", MultiDOFJointTrajectory, queue_size=10)

        for t in range(100):




        # Main while loop.
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg:
                pub_dji.publish(self.dji_msg)
                self.got_new_msg = False
            r.sleep()

    # Odometry callback function.
    def rpy_callback(self, msg):
        self.dji_msg.axes[0] = msg.roll
        self.dji_msg.axes[1] = msg.pitch
        self.dji_msg.axes[2] = msg.thrust.z
        self.dji_msg.axes[3] = msg.yaw_rate
        self.dji_msg.header.stamp = rospy.Time.now()
        self.got_new_msg = True


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('rpy_convert_node')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        rpy_convert_node = RPYConvertNode()
    except rospy.ROSInterruptException: pass
