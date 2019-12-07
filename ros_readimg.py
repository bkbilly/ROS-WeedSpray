#!/usr/bin/env python

# Python libs
import sys
import time

# OpenCV
import cv2
from cv2 import imshow

# Ros libraries
import roslib
import rospy
import image_geometry

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import copy

import time
import numpy as np

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# My libs
from canopy import CanopyClass

def movebase_client(x, y, z):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        print('Done moving to: {}, {}, {}'.format(x, y, z))
        # time.sleep(3)
        return client.get_result()

class image_projection(CanopyClass):
    camera_model = None
    inputimage = None

    def __init__(self, robot, inputimage=None):
        self.inputimage = inputimage
        self.robot = robot
        self.image_pub = rospy.Publisher(
            "/opencv/image_raw/{}".format(self.robot),
            Image,
            queue_size=5)
        self.contours_pub = rospy.Publisher(
            "/opencv/contours/{}".format(self.robot),
            PoseStamped)

        self.bridge = CvBridge()

        self.camera_info_sub = rospy.Subscriber(
            '/%s/kinect2_camera/hd/camera_info' % (self.robot),
            CameraInfo,
            self.camera_info_callback)

        rospy.Subscriber(
            "/%s/kinect2_camera/hd/image_color_rect" % (self.robot),
            Image, self.image_callback)


    def publish_contours(self, contours):
        self.image_pub.publish(contours)

    def image_callback(self, data):
        if not self.camera_model:
            return

        if self.inputimage is None:
            self.image_pub.publish(data)
            return 0

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


        # ground, ground_mask = self.filter_colors(cv_image, 'ground')
        ground_inv, ground_inv_mask = self.filter_colors(cv_image, 'ground_inv')
        plant, plant_mask = self.filter_colors(ground_inv, self.inputimage)
        contours_image, contours, contours_boxes, contours_points = self.get_contours(plant, cv_image)
        # self.publish_contours(contours)

        contours_s = cv2.resize(contours_image, (0, 0), fx=0.5, fy=0.5)

        # Publish new image
        self.image_pub.publish(
            self.bridge.cv2_to_imgmsg(contours_s, encoding="bgr8"))

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister()  # Only subscribe once


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_projection', anonymous=True)
    img_proj = image_projection('thorvald_001')
    # image_projection('thorvald_002')

    img_proj.inputimage = 'simple_inv'
    movebase_client(6, -3.8, 90)
    movebase_client(-6, -3.8, 90)
    movebase_client(-6, -2.7, 0)
    movebase_client(6, -2.7, 0)
    img_proj.inputimage = None

    img_proj.inputimage = 'realeasy_inv'
    movebase_client(6, -0.7, 90)
    movebase_client(-6, -0.7, 90)
    movebase_client(-6, 0.2, 0)
    movebase_client(6, 0.2, 0)
    img_proj.inputimage = None

    img_proj.inputimage = 'realhard_inv'
    movebase_client(6, 2.2, 90)
    movebase_client(-6, 2.2, 90)
    movebase_client(-6, 3.2, 0)
    movebase_client(6, 3.2, 0)
    img_proj.inputimage = None

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
