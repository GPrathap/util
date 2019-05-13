#!/usr/bin/python
# -*- coding: utf-8 -*-
PKG = 'dji_command'
import roslib; roslib.load_manifest(PKG)

import rospy
import yaml
from sensor_msgs.msg import CameraInfo

def yaml_to_CameraInfo(yaml_fname):
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

if __name__ == "__main__":
    # Get fname from command line (cmd line input required)

    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=True)
    publisher = rospy.Publisher("/pylon_camera_node_aca1300/camera_info", CameraInfo, queue_size=10)

    import argparse
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("filename", help="Path to yaml file containing " +\
                                             "camera calibration data")
    args, unknown = arg_parser.parse_known_args()
    # args = arg_parser.parse_args()
    filename = args.filename

    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(filename)
    # camera_info_msg = yaml_to_CameraInfo("/home/roman/Dropbox/Projects/InnoDrone/camera.yaml")


    rate = rospy.Rate(10)

    # Run publisher
    while not rospy.is_shutdown():
        camera_info_msg.header.frame_id = "pylon_camera"
        camera_info_msg.header.stamp = rospy.Time.now()
        publisher.publish(camera_info_msg)
        rate.sleep()
