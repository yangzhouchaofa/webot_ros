#!/usr/bin/env python

# 发布/cmd_vel，并接收轮子的转速
# 用于测试发布话题与轮子实际速度用的代码
import math
from time import time

import rospy
from webots_ros.srv import set_float, set_floatRequest, get_float, get_floatRequest
from geometry_msgs.msg import Twist, PointStamped

motorNames = ['wheel_lm_motor', 'wheel_rm_motor']
gps_names = ['gpsFront', 'gpsBack']

vel = [0.0, -0.0]


def velocity_publisher():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 1  # 设置线性速度 (m/s)
    cmd_vel_msg.angular.z = 0  # 设置角速度 (rad/s)
    pub.publish(cmd_vel_msg)  # 发布消息


def velocity_get_server():
    for i in range(len(motorNames)):
        rospy.wait_for_service('/robot/' + motorNames[i] + '/get_velocity')
        get_vel_service = rospy.ServiceProxy('/robot/' + motorNames[i] + '/get_velocity', get_float)
        get_vel_requests = get_floatRequest()
        vel[i] = float(str(get_vel_service(get_vel_requests))[7:])
        print(vel)


def tf():

    xyz = rospy.wait_for_message('/robot/gpsFront/values', PointStamped)
    x = xyz.point.x
    y = xyz.point.z

    return x, y


def main():
    rospy.init_node('vel', anonymous=True)
    Xlist = [0]
    Ylist = [0]
    timelast = time()
    velocity_publisher()
    while not rospy.is_shutdown():
        x, y = tf()
        timenow = time()
        Xlist.append(x)
        Ylist.append(y)
        dx = Xlist[-1] - Xlist[-2]
        dy = Ylist[-1] - Ylist[-2]
        dt = timenow - timelast
        disReward = math.sqrt(dx * dx + dy * dy)
        v = disReward / dt
        print("real:",v)
        velocity_publisher()
        timelast = timenow


if __name__ == '__main__':
    main()
