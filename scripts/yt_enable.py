#!/usr/bin/env python

import rospy
from webots_ros.srv import set_int, set_intRequest

gps_names = ['gpsFront','gpsBack']
lidar_names = ['rslidar']
gps_sample_period = 100
lidar_sample_period = 100

class ENABLE():
    def __init__(self, names, sample_period):
        self.names = names
        self.sample_period = sample_period

    def enable(self):
        for i in range(len(self.names)):
            set_gps_client = rospy.ServiceProxy('/robot/'+self.names[i]+'/enable', set_int)
            enable_gps_req = set_intRequest()
            enable_gps_req.value = self.sample_period
            enable_gps_resp = set_gps_client(enable_gps_req)
            rospy.loginfo(enable_gps_resp)

def main():
    rospy.init_node('yt_enable', anonymous=True)

    yt_gps = ENABLE(gps_names, gps_sample_period)
    yt_gps.enable()
    yt_lidar = ENABLE(lidar_names, lidar_sample_period)
    yt_lidar.enable()


if __name__ == '__main__':
    main()
