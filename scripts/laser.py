#!/usr/bin/env python

import rospy
from forSAC import recommand

PICKLE = 'laser.pkl'


def main():
    rospy.init_node('laser', anonymous=True)
    gpsback = recommand(PICKLE, 'laser')
    gpsback.wait_for_plan()
    gpsback.receive_laser()
    gpsback.save()


if __name__ == '__main__':
    main()
