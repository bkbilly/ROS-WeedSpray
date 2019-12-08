#!/usr/bin/env python

# Python libs
import sys
import math

# Ros libraries
import rospy

# Ros Messages
from sensor_msgs.msg import PointCloud


class image_projection():

    def __init__(self, robot):
        self.robot = robot
        self.keeplist = []

        self.camera_info_sub = rospy.Subscriber(
            "/weed/points/{}".format(self.robot),
            PointCloud,
            self.points_callback)

    def points_callback(self, data):
        # print(data)
        for point in data.points:
            new_position = (point.x, point.y, point.z)
            # print(new_position)
            found_close = False
            for keep in self.keeplist:
                dx = abs(new_position[0] - keep[0])
                dy = abs(new_position[1] - keep[1])
                # dz = abs(new_position[2] - keep[2])
                # dist = dx + dy + dz
                dist = math.hypot(dx, dy)
                if dist < 0.1:
                    found_close = True

            if not found_close:
                self.keeplist.append(new_position)
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
