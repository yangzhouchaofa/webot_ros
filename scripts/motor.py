#!/usr/bin/env python

import rospy
from forSAC import recommand

PICKLE = 'motor.pkl'


def main():
    rospy.init_node('motor', anonymous=True)
    lm = recommand(PICKLE, 'motor')
    lm.wait_for_plan()
    lm.receive_motor()
    lm.save()
    lm.save_obs()


if __name__ == '__main__':
    main()