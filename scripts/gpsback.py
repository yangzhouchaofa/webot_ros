#!/usr/bin/env python

import rospy
from forSAC import recommand

PICKLE = 'gpsback.pkl'


def main():
    rospy.init_node('gpsback', anonymous=True)
    gpsback = recommand(PICKLE, 'gpsBack')
    gpsback.wait_for_plan()
    gpsback.receive_gps()
    gpsback.save()


if __name__ == '__main__':
    main()