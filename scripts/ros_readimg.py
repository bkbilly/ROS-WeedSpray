#!/usr/bin/env python

# Python libs
import sys

# OpenCV
import cv2

# Ros libraries
import rospy
import image_geometry
import tf

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from geometry_msgs.msg import PointStamped, Point32
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

# My libs
from canopy import CanopyClass


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

        rospy.Subscriber(
            "/weed/row/{}".format(self.robot),
            String,
            self.changerow_callback)

        self.point_msg = PointStamped()
        self.points_msg = PointCloud()
        self.tflistener = tf.listener.TransformListener()

    def changerow_callback(self, data):
        if self.inputimage != data.data:
            print('Changed detection row to: {}'.format(data))
            self.inputimage = data.data

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister()  # Only subscribe once

    def publish_points(self, contours):
        # print('Found points: {}'.format(len(contours)))
        time = rospy.Time(0)
        self.points_msg.points = []
        self.points_msg.header.frame_id = self.camera_model.tfFrame()
        self.points_msg.header.stamp = time
        for cnt in contours:
            rect = self.camera_model.rectifyPoint(cnt)
            x, y, z = self.camera_model.projectPixelTo3dRay(rect)
            x *= 0.493
            y *= 0.493
            z = 0.493
            if -0.03 <= x <= 0.03:
                self.points_msg.points.append(Point32(x, y, z))
        print('Found filtered points: {}'.format(len(self.points_msg.points)))

        tf_points = self.tflistener.transformPointCloud('map', self.points_msg)
        # print(tf_points)
        self.contours_pub.publish(tf_points)

    def image_callback(self, data):
        if not self.camera_model:
            return

        if self.inputimage is None or self.inputimage == '':
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

        contours_s = cv2.resize(contours_image, (0, 0), fx=0.5, fy=0.5)

        # Publish new image
        self.image_pub.publish(
            self.bridge.cv2_to_imgmsg(contours_s, encoding="bgr8"))


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_projection', anonymous=True)
    image_projection('thorvald_001', 'realhard_inv')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)
