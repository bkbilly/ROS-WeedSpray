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

import time
import numpy as np

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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

class image_projection:
    camera_model = None
    inputimage = None

    def __init__(self, robot, inputimage=None):
        self.inputimage = inputimage
        self.robot = robot
        self.image_pub = rospy.Publisher(
            "/output/image_raw/compressed",
            Image,
            queue_size=5)

        self.bridge = CvBridge()

        self.camera_info_sub = rospy.Subscriber(
            '/%s/kinect2_camera/hd/camera_info' % (self.robot),
            CameraInfo,
            self.camera_info_callback)

        rospy.Subscriber(
            "/%s/kinect2_camera/hd/image_color_rect" % (self.robot),
            Image, self.image_callback)

    def image_callback(self, data):
        if not self.camera_model:
            return

        if self.inputimage is None:
            self.image_pub.publish(data)
            return 0

        # project a point in camera coordinates into the pixel coordinates
        # uv = self.camera_model.project3dToPixel((0, 0, 0.5))

        # print('Running with image: {}'.format(self.inputimage))
        # print 'Pixel coordinates: ', uv
        # print ''

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # inputimage = 'simple.png'
        # cv_image = cv2.imread(inputimage)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # img_simple = cv2.imread('img_simple.png', cv2.IMREAD_GRAYSCALE)
        # img_realeasy = cv2.imread('img_realeasy.png', cv2.IMREAD_GRAYSCALE)
        # img_realhard = cv2.imread('img_realhard.png', cv2.IMREAD_GRAYSCALE)

        # orb = cv2.ORB()
        # kp1, des1 = orb.detectAndCompute(gray_image, None)
        # kp2, des2 = orb.detectAndCompute(img_realeasy, None)
        # bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        # matches = bf.match(des1, des2)
        # matches = sorted(matches, key=lambda x: x.distance)
        # print(len(matches))

        if self.inputimage == 'simple':
            hsv = cv2.blur(hsv, (100, 100))
            lower_filter = np.array([10, 120, 20])
            upper_filter = np.array([60, 190, 130])
        elif self.inputimage == 'realeasy':
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            hsv = cv2.blur(hsv, (100, 100))
            lower_filter = np.array([30, 100, 40])
            upper_filter = np.array([60, 170, 130])
        elif self.inputimage == 'realhard':
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            hsv = cv2.blur(hsv, (10, 10))
            lower_filter = np.array([30, 0, 20])
            upper_filter = np.array([255, 255, 255])

        mask = cv2.inRange(hsv, lower_filter, upper_filter)
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # cv2.imshow('hsv cv_image', hsv)
        # cv2.imshow('res cv_image', res)
        # cv2.waitKey(0)

        # Grayscale
        gray_res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('Canny Edges After Contouring', gray_res)

        # Find Canny edges
        # edged = cv2.Canny(gray_res, 150, 150)
        ret, edged = cv2.threshold(gray_res, 40, 255, 0)

        # Finding Contours
        im2, contours, h = cv2.findContours(
            edged,
            cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # cv2.imshow('Canny Edges After Contouring', edged)

        # Draw all contours
        # -1 signifies drawing all contours
        cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 3)
        cv_image_s = cv2.resize(cv_image, (0, 0), fx=0.5, fy=0.5)

        # msg = CompressedImage()
        # msg.header.stamp = rospy.Time.now()
        # msg.format = "jpeg"
        # msg.data = self.bridge.cv2_to_imgmsg(cv_image_s)
        # Publish new image
        self.image_pub.publish(
            self.bridge.cv2_to_imgmsg(cv_image_s, encoding="bgr8"))

        # cv2.imshow('Contours', cv_image_s)
        # cv2.waitKey(0)

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister()  # Only subscribe once


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_projection', anonymous=True)
    img_proj = image_projection('thorvald_001')
    # image_projection('thorvald_002')

    img_proj.inputimage = 'simple'
    movebase_client(6, -3.8, 90)
    movebase_client(-6, -3.8, 90)
    img_proj.inputimage = None
    movebase_client(-6, -2.7, 0)
    movebase_client(6, -2.7, 0)
    img_proj.inputimage = None

    img_proj.inputimage = 'realeasy'
    movebase_client(6, -0.7, 90)
    movebase_client(-6, -0.7, 90)
    img_proj.inputimage = None
    movebase_client(-6, 0.2, 0)
    movebase_client(6, 0.2, 0)
    img_proj.inputimage = None

    img_proj.inputimage = 'realhard'
    movebase_client(6, 2.2, 90)
    movebase_client(-6, 2.2, 90)
    img_proj.inputimage = None
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

