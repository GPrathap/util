#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function


import roslib
import rospy

import sensor_msgs.msg
import geometry_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs import point_cloud2

from sensor_msgs.msg import PointCloud2, PointField

import tf2_ros
import tf2_geometry_msgs

# Python libs
import sys, time

# numpy and scipy
import numpy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2


from image_geometry import PinholeCameraModel, StereoCameraModel


import struct

# def xyzrgb_array_to_pointcloud2(points, colors, stamp=None, frame_id=None, seq=None):
#     '''
#     Create a sensor_msgs.PointCloud2 from an array
#     of points.
#     '''
#     msg = PointCloud2()
#     assert(points.shape == colors.shape)
#
#     buf = []
#
#     if stamp:
#         msg.header.stamp = stamp
#     if frame_id:
#         msg.header.frame_id = frame_id
#     if seq:
#         msg.header.seq = seq
#     if len(points.shape) == 3:
#         msg.height = points.shape[1]
#         msg.width = points.shape[0]
#     else:
#         N = len(points)
#         xyzrgb = np.array(np.hstack([points, colors]), dtype=np.float32)
#         msg.height = 1
#         msg.width = N
#
#     msg.fields = [
#         PointField('x', 0, PointField.FLOAT32, 1),
#         PointField('y', 4, PointField.FLOAT32, 1),
#         PointField('z', 8, PointField.FLOAT32, 1),
#         PointField('r', 12, PointField.FLOAT32, 1),
#         PointField('g', 16, PointField.FLOAT32, 1),
#         PointField('b', 20, PointField.FLOAT32, 1)
#     ]
#     msg.is_bigendian = False
#     msg.point_step = 24
#     msg.row_step = msg.point_step * N
#     msg.is_dense = True;
#     msg.data = xyzrgb.tostring()
#
#     return msg





class ProjectPoints():
    def __init__(self):
        self.ci = sensor_msgs.msg.CameraInfo()
        self.pc = sensor_msgs.msg.PointCloud2()
        self.im = sensor_msgs.msg.CompressedImage()

        sub_ci = rospy.Subscriber("/pylon_camera_node_aca1300/camera_info", sensor_msgs.msg.CameraInfo, self.camera_info_callback)
        sub_pc = rospy.Subscriber("/velodyne_points_camfov", sensor_msgs.msg.PointCloud2, self.point_cloud_callback)
        sub_im = rospy.Subscriber("/pylon_camera_node_aca1300/image_raw/compressed", sensor_msgs.msg.CompressedImage, self.image_callback)

        self.pub_pc = rospy.Publisher("/velodyne_points_camfov_colored", sensor_msgs.msg.PointCloud2, queue_size=10)


        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.got_ci = False
        self.got_pc = False
        self.got_im = False

        while not rospy.is_shutdown():
            if self.got_ci and self.got_pc and self.got_im:
                self.got_pc = False

                # print( self.pc.width * self.pc.height )
                    # self.got_new_msg = False

                # print("self.pc.header.frame_id", self.pc.header.frame_id )

                transform = self.tf_buffer.lookup_transform(self.ci.header.frame_id,
                                                        self.pc.header.frame_id, #source frame
                                                        rospy.Time(0), #get the tf at first available time
                                                        rospy.Duration(0.1)) #wait for 1 second

                #### direct conversion to CV2 ####
                np_arr = np.fromstring(self.im.data, np.uint8)
                # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
                image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

                # points = np.array([])
                # colors = np.array([])
                points = []
                cam = PinholeCameraModel()
                cam.fromCameraInfo(self.ci)
                
                for p in pc2.read_points(self.pc, field_names = ("x", "y", "z"), skip_nans=True):
                    v = geometry_msgs.msg.Vector3Stamped()
                    v.vector.x = p[0]
                    v.vector.y = p[1]
                    v.vector.z = p[2]
                    out = tf2_geometry_msgs.do_transform_vector3(v, transform)


                    im_point = cam.project3dToPixel((out.vector.x, out.vector.y, out.vector.z))

                    # cv2.circle(image_np,(int(im_point[0]),int(im_point[1])), 3, (0,0,255), -1)

                    if(int(im_point[0]) > 0 and int(im_point[0]) < 1280):
                        [b,g,r] = image_np[int(im_point[1]),int(im_point[0])]
                        # cv2.circle(image_np,(int(im_point[0]),int(im_point[1])), 3,  np.array((int(b),int(g),int(r))), -1)
                        a = 255

                        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                        # pt = [p[0], p[1], p[2], rgb]
                        pt = [out.vector.x, out.vector.y, out.vector.z, rgb]
                        points.append(pt)

                fields = [PointField('x', 0, PointField.FLOAT32, 1),PointField('y', 4, PointField.FLOAT32, 1),PointField('z', 8, PointField.FLOAT32, 1),PointField('rgba', 12, PointField.UINT32, 1)]
                header = self.ci.header
                pc2res = point_cloud2.create_cloud(header, fields, points)


                # cv2.imshow('cv_img', image_np)
                # cv2.waitKey(1)

                self.pub_pc.publish(pc2res)




    def camera_info_callback(self, msg):
        self.ci = msg
        self.got_ci = True

    def point_cloud_callback(self, msg):
        self.pc = msg
        self.got_pc = True

    def image_callback(self, ros_data):
        self.im = ros_data
        self.got_im = True
        # VERBOSE = True
        # if VERBOSE:
        #     print ("received image of type: ", ros_data.format)
        #
        # #### direct conversion to CV2 ####
        # np_arr = np.fromstring(ros_data.data, np.uint8)
        # # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        #
        # #### Feature detectors using CV2 ####
        # # "","Grid","Pyramid" +
        # # "FAST","GFTT","HARRIS","MSER","ORB","SIFT","STAR","SURF"
        # method = "GridFAST"
        # feat_det = cv2.ORB_create()
        # time1 = time.time()
        #
        # # convert np image to grayscale
        # featPoints = feat_det.detect(
        #     cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY))
        # time2 = time.time()
        # # if VERBOSE :
        # #     print '%s detector found: %s points in: %s sec.'%(method,
        # #         len(featPoints),time2-time1)
        #
        # for featpoint in featPoints:
        #     x,y = featpoint.pt
        #     cv2.circle(image_np,(int(x),int(y)), 3, (0,0,255), -1)
        #
        # cv2.imshow('cv_img', image_np)
        # cv2.waitKey(1)

        # #### Create CompressedIamge ####
        # msg = CompressedImage()
        # msg.header.stamp = rospy.Time.now()
        # msg.format = "jpeg"
        # msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # # Publish new image
        # self.image_pub.publish(msg)


    # def test_monocular(self):
    #     ci = sensor_msgs.msg.CameraInfo()
    #     ci.width = 640
    #     ci.height = 480
    #     print(ci)
    #     cam = PinholeCameraModel()
    #     cam.fromCameraInfo(ci)
    #     print(cam.rectifyPoint((0, 0)))
    #
    #     print(cam.project3dToPixel((0,0,0)))


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('project_points')
    try:
        project_points = ProjectPoints()
    except rospy.ROSInterruptException: pass
