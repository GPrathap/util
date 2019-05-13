#!/usr/bin/python
# -*- coding: utf-8 -*-

# Start up ROS pieces.
PKG = 'dji_command'
import roslib; roslib.load_manifest(PKG)
import rospy
import std_msgs

# ROS messages.
from sensor_msgs.msg import Joy
from mav_msgs.msg import RollPitchYawrateThrust

class RPYConvertNode():
    def __init__(self):
        self.got_new_msg = False
        self.dji_msg = Joy()
        self.dji_msg.axes = [0, 0, 0, 0, 0]
        self.dji_msg.axes[4] = 0x20 | 0x00 | 0x08 | 0x02 #(VERTICAL_THRUST | HORIZONTAL_ANGLE | YAW_RATE | HORIZONTAL_BODY);
        self.dji_msg.header = std_msgs.msg.Header()

        # Create subscribers and publishers.
        sub_mav_rpy = rospy.Subscriber("/command/roll_pitch_yawrate_thrust", RollPitchYawrateThrust, self.rpy_callback)
        # pub_dji = rospy.Publisher("/dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", Joy, queue_size=10)
        pub_dji = rospy.Publisher("/dji_sdk/flight_control_setpoint_generic", Joy, queue_size=10)

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
