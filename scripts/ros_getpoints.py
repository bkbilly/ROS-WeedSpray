#!/usr/bin/env python

# Python libs
import sys
import math

# Ros libraries
import rospy
import tf
from std_srvs.srv import Empty

# Ros Messages
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry


class image_projection():

    def __init__(self, robot):
        self.robot = robot
        self.keeplist = []
        self.notsprayed = []
        self.spr = rospy.ServiceProxy(
            "{}/spray".format(self.robot),
            Empty)

        rospy.Subscriber(
            "/weed/points/{}".format(self.robot),
            PointCloud,
            self.points_callback)

        rospy.Subscriber(
            "/{}/odometry/base_raw".format(self.robot),
            Odometry,
            self.odometry_callback)

        self.points_pub = rospy.Publisher(
            "/weed/allpoints/{}".format(self.robot),
            PointCloud,
            queue_size=5)
        self.points_msg = PointCloud()

        self.tflistener = tf.listener.TransformListener()

    def publish_allpoints(self):
        time = rospy.Time(0)
        self.points_msg.points = self.keeplist
        self.points_msg.header.frame_id = 'map'
        self.points_msg.header.stamp = time

        self.points_pub.publish(self.points_msg)

    def odometry_callback(self, data):
        try:
            trans, rot = self.tflistener.lookupTransform(
                'map',
                '{}/sprayer'.format(self.robot),
                data.header.stamp)
            # print(trans, rot)
        except Exception:
            return

        newkeep = []
        shouldSpray = False
        for keep in self.notsprayed:
            dx = abs(trans[0] - keep.x)
            # print(dx)
            if dx < 0.07:
                shouldSpray = True
            else:
                newkeep.append(keep)

        if shouldSpray:
            self.notsprayed = newkeep
            print('spray!!!')
            self.spr()

        self.publish_allpoints()

    def points_callback(self, data):
        hasChanged = False
        for point in data.points:
            # check if point has already been added
            found_close = False
            for keep in self.keeplist:
                dx = abs(point.x - keep.x)
                dy = abs(point.y - keep.y)
                dist = math.hypot(dx, dy)
                if dist < 0.07:
                    found_close = True

            # Not found on our list, append it
            if not found_close:
                hasChanged = True
                self.keeplist.append(point)
                self.notsprayed.append(point)

        if hasChanged:
            print('Found points: {}'.format(len(self.keeplist)))
        # print(self.keeplist)


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('get_points', anonymous=True)
    image_projection('thorvald_001')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)
