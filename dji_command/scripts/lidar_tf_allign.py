#!/usr/bin/env python

import rospy
import tf
import math


from dynamic_reconfigure.server import Server
from dji_command.cfg import LidarAllignConfig


class LidarAllign:
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.z = 0
        self.configsrv = Server(LidarAllignConfig, self.configCallback)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timerCallback)
        rospy.spin()


    def configCallback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {roll}, {pitch}, {yaw}, {z}""".format(**config))
        self.roll = config.roll
        self.pitch = config.pitch
        self.yaw = config.yaw
        self.z = config.z
        return config

    def timerCallback(self, event):
        br = tf.TransformBroadcaster()
        theta = 0
        br.sendTransform((0, 0, self.z),
                         tf.transformations.quaternion_from_euler(math.radians(90+self.pitch), math.radians(180+self.roll), math.radians(90+self.yaw)),
                         rospy.Time.now(),
                         "pylon_camera",
                         "velodyne")

if __name__ == "__main__":
    rospy.init_node("lidar_tf_allign", anonymous = True)
    try:
        lidar_allign = LidarAllign()
    except rospy.ROSInterruptException: pass
