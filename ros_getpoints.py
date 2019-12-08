#!/usr/bin/env python

# Python libs
import sys

# Ros libraries
import rospy
import tf

# Ros Messages
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError


class image_projection():

    def __init__(self, robot):
        self.robot = robot
        self.keeplist = []

        self.camera_info_sub = rospy.Subscriber(
            "/weed/point/{}".format(self.robot),
            PointStamped,
            self.points_callback)


    def points_callback(self, data):
        new_position = (data.pose.position.x, data.pose.position.y)
        # print(new_position)
        found_close = False
        for keep in self.keeplist:
            dx = abs(new_position[0] - keep[0])
            dy = abs(new_position[1] - keep[1])
            dist = dx + dy
            if dist < 0.2:
                found_close = True

        if not found_close:
            self.keeplist.append(new_position)
        print('Found points: {}'.format(len(self.keeplist)))
        # print(self.keeplist)


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('get_points', anonymous=True)
    img_proj = image_projection('thorvald_001')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)
