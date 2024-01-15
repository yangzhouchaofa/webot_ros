#!/usr/bin/env python
from time import sleep

import rospy
import tf
from move_base_msgs.msg import MoveBaseActionGoal
from webots_ros.srv import set_float, set_floatRequest, get_float, get_floatRequest




def main():
    rospy.init_node('yt_position', anonymous=True)
    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
    msg = MoveBaseActionGoal()
    roll = 0.0
    pitch = 0.0
    yaw = 0  # 90度，单位：弧度

    # 将欧拉角转换为四元数
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    print(quaternion)
    msg.goal.target_pose.header.frame_id = "map"
    msg.goal.target_pose.pose.position.x = 0
    msg.goal.target_pose.pose.position.y = 0
    msg.goal.target_pose.pose.position.z = 0
    msg.goal.target_pose.pose.orientation.x = quaternion[0]
    msg.goal.target_pose.pose.orientation.y = quaternion[1]
    msg.goal.target_pose.pose.orientation.z = quaternion[2]
    msg.goal.target_pose.pose.orientation.w = quaternion[3]
    sleep(2)
    while True:
        pub.publish(msg)  # 发布消息
        print("pub")



if __name__ == '__main__':
    main()
