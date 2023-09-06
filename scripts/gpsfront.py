#!/usr/bin/env python

import rospy
from forSAC import recommand

PICKLE = 'gpsfront.pkl'

def main():
    rospy.init_node('gpsfront', anonymous=True)
    gpsfront = recommand(PICKLE, 'gpsFront')
    gpsfront.wait_for_plan()
    gpsfront.receive_gps()
    gpsfront.save()


if __name__ == '__main__':
    main()
