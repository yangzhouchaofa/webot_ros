#!/usr/bin/env python

import math

import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped, PointStamped

gps_names = ['gpsFront','gpsBack']

class YT_TF_STATIC():
    def __init__(self, name):
        self.x = 0
        self.y = 0
        self.z = 0
        self.qtn = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tfs = TransformStamped()
        # --- 头信息
        self.tfs.header.frame_id = "map"
        self.tfs.header.stamp = rospy.Time.now()
        self.tfs.header.seq = 101
        # --- 子坐标系
        self.tfs.child_frame_id = name


    def write_tf(self, x, y, z, angle):
        self.tfs.transform.translation.x = x
        self.tfs.transform.translation.y = y
        self.tfs.transform.translation.z = z
        self.qtn = tf.transformations.quaternion_from_euler(angle, 0, 0)  # --p 欧拉角转换成四元数
        self.tfs.transform.rotation.x = self.qtn[0]
        self.tfs.transform.rotation.y = self.qtn[1]
        self.tfs.transform.rotation.z = self.qtn[2]
        self.tfs.transform.rotation.w = self.qtn[3]
        self.broadcaster.sendTransform(self.tfs)



class YT_TF():
    def __init__(self, names):
        self.names = names
        self.xyz = [0,0]
        self.x = 0
        self.y = 0
        self.z = 0
        self.qtn = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.tfs = TransformStamped()
        # --- 头信息
        self.tfs.header.frame_id = "map"
        self.tfs.header.stamp = rospy.Time.now()
        self.tfs.header.seq = 101
        # --- 子坐标系
        self.tfs.child_frame_id = "base_link"  # 这里表示雷达坐标系

    def read_gps(self):
        for i in range(len(self.names)):
            self.xyz[i] = rospy.wait_for_message('/robot/'+ self.names[i]+'/values',PointStamped)
        self.x = (self.xyz[1].point.x + self.xyz[0].point.x) / 2
        self.z = (self.xyz[1].point.y + self.xyz[0].point.y) / 2
        self.y = (self.xyz[1].point.z + self.xyz[0].point.z) / 2


    def write_tf(self):
        self.tfs.header.stamp = rospy.Time.now()
        self.tfs.transform.translation.x = self.x
        self.tfs.transform.translation.y = self.y
        self.tfs.transform.translation.z = self.z
        self.angle = math.atan2(self.xyz[0].point.z - self.xyz[1].point.z, self.xyz[0].point.x - self.xyz[1].point.x)
        self.qtn = tf.transformations.quaternion_from_euler(0, 0, self.angle)  # --p 欧拉角转换成四元数
        self.tfs.transform.rotation.x = self.qtn[0]
        self.tfs.transform.rotation.y = self.qtn[1]
        self.tfs.transform.rotation.z = self.qtn[2]
        self.tfs.transform.rotation.w = self.qtn[3]
        self.broadcaster.sendTransform(self.tfs)



def main():
    rospy.init_node('yt_tf', anonymous=True)
    yt = YT_TF(gps_names)
    odom = YT_TF_STATIC('odom')
    odom.write_tf(0,0,0,0)
    while not rospy.is_shutdown():
        yt.read_gps()
        yt.write_tf()



if __name__ == '__main__':
    main()

