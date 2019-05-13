#!/usr/bin/env python
import roslib
roslib.load_manifest('geodetic_utils')
import rospy
import geometry_msgs
from nav_msgs.msg import Odometry
import math

import tf

gimbal_ang = geometry_msgs.msg.Vector3()

def handle_gimbal_angle(msg):
    global gimbal_ang
    gimbal_ang = msg.vector
    # br = tf.TransformBroadcaster()
    # br.sendTransform((0, 0, 0),
    #                  tf.transformations.quaternion_from_euler(0, 0, 0),
    #                  rospy.Time.now(),
    #                  "velodyne",
    #                  "gps_receiver")
    return


def handle_odometry(msg):
    # print(gimbal_ang.x)
    orientation_q = msg.pose.pose.orientation
    euler_drone = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    roll_drone = euler_drone[0]
    pitch_drone = euler_drone[1]
    yaw_drone = euler_drone[2]

    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     # roll, pitch and yaw angles
                     # tf.transformations.quaternion_from_euler(math.pi*gimbal_ang.x/180.0, math.pi*gimbal_ang.y/180.0, math.pi*gimbal_ang.z/180.0),
                     tf.transformations.quaternion_from_euler(0 * math.pi/180.0,
                                -gimbal_ang.x * math.pi/180.0, yaw_drone - gimbal_ang.z * math.pi/180.0),
                     rospy.Time.now(),
                     "velodyne",
                     "world")


if __name__ == '__main__':
    rospy.init_node('gimbal_tf_broadcaster')
    rospy.Subscriber('dji_sdk/gimbal_angle',
                     geometry_msgs.msg.Vector3Stamped,
                     handle_gimbal_angle)

    rospy.Subscriber('dji_sdk/odometry',
                     Odometry,
                     handle_odometry)

    rospy.spin()
