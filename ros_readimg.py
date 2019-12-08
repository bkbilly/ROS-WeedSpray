#!/usr/bin/env python

# Python libs
import sys

# OpenCV
import cv2

# Ros libraries
import rospy
import image_geometry

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from geometry_msgs.msg import PointStamped, Point32
from cv_bridge import CvBridge, CvBridgeError

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

# My libs
from canopy import CanopyClass


def movebase_client(x, y, z):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
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
        return client.get_result()


class image_projection(CanopyClass):
    camera_model = None
    inputimage = None

    def __init__(self, robot, inputimage=None):
        self.inputimage = inputimage
        self.robot = robot
        self.image_pub = rospy.Publisher(
            "/weed/spray/{}".format(self.robot),
            Image,
            queue_size=5)

        self.contour_pub = rospy.Publisher(
            "/weed/point/{}".format(self.robot),
            PointStamped,
            queue_size=5)

        self.contours_pub = rospy.Publisher(
            "/weed/points/{}".format(self.robot),
            PointCloud,
            queue_size=5)

        self.bridge = CvBridge()

        self.camera_info_sub = rospy.Subscriber(
            '/%s/kinect2_camera/hd/camera_info' % (self.robot),
            CameraInfo,
            self.camera_info_callback)

        rospy.Subscriber(
            "/%s/kinect2_camera/hd/image_color_rect" % (self.robot),
            Image, self.image_callback)

        self.point_msg = PointStamped()
        self.points_msg = PointCloud()
        self.tflistener = tf.listener.TransformListener()

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister()  # Only subscribe once

    def publish_points(self, contours):
        print('Found points: {}'.format(len(contours)))
        time = rospy.Time(0)
        self.points_msg.points = []
        self.points_msg.header.frame_id = self.camera_model.tfFrame()
        self.points_msg.header.stamp = time
        for cnt in contours:
            rect = self.camera_model.rectifyPoint(cnt)
            x, y, z = self.camera_model.projectPixelTo3dRay(rect)
            self.points_msg.points.append(Point32(x, y, z))
        print(self.points_msg.points)

        # self.tflistener.lookupTransform(self.camera_model.tfFrame(), 'map', time)
        tf_points = self.tflistener.transformPointCloud('map', self.points_msg)
        self.contours_pub.publish(tf_points)

    def publish_point(self, contours):
        print('Found points: {}'.format(len(contours)))
        for cnt in contours:
            time = rospy.Time(0)
            rect = self.camera_model.rectifyPoint(cnt)
            camera_point = self.camera_model.projectPixelTo3dRay(rect)
            self.point_msg.point.x = camera_point[0]
            self.point_msg.point.y = camera_point[1]
            self.point_msg.point.z = camera_point[2]
            self.point_msg.header.frame_id = self.camera_model.tfFrame()
            self.point_msg.header.stamp = time

            # print(self.point_msg)
            self.tflistener.lookupTransform(self.camera_model.tfFrame(), 'map', time)
            tf_point = self.tflistener.transformPoint('map', self.point_msg)
            print(tf_point)

            print(tf_point.point.x, tf_point.point.y)
            self.contour_pub.publish(tf_point)

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
        self.publish_points(contours_points)
        # self.publish_point(contours_points)

        contours_s = cv2.resize(contours_image, (0, 0), fx=0.5, fy=0.5)

        # Publish new image
        self.image_pub.publish(
            self.bridge.cv2_to_imgmsg(contours_s, encoding="bgr8"))


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_projection', anonymous=True)
    img_proj = image_projection('thorvald_001')
    # image_projection('thorvald_002')

    img_proj.inputimage = None
    img_proj.inputimage = 'simple_inv'
    # movebase_client(6, -3.8, 90)
    # movebase_client(-6, -3.8, 90)
    # movebase_client(-6, -2.7, 0)
    # movebase_client(6, -2.7, 0)
    # img_proj.inputimage = None

    # img_proj.inputimage = 'realeasy_inv'
    # movebase_client(6, -0.7, 90)
    # movebase_client(-6, -0.7, 90)
    # movebase_client(-6, 0.2, 0)
    # movebase_client(6, 0.2, 0)
    # img_proj.inputimage = None

    # img_proj.inputimage = 'realhard_inv'
    # movebase_client(6, 2.2, 90)
    # movebase_client(-6, 2.2, 90)
    # movebase_client(-6, 3.2, 0)
    # movebase_client(6, 3.2, 0)
    # img_proj.inputimage = None

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)
