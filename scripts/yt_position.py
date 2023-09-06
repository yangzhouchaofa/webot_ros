#!/usr/bin/env python
from time import sleep

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from webots_ros.srv import set_float, set_floatRequest, get_float, get_floatRequest





def main():
    rospy.init_node('yt_position', anonymous=True)
    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
    msg = MoveBaseActionGoal()
    msg.goal.target_pose.header.frame_id = "map"
    msg.goal.target_pose.pose.position.x = 3.2
    msg.goal.target_pose.pose.position.y = 0.5
    msg.goal.target_pose.pose.position.z = 0
    msg.goal.target_pose.pose.orientation.x = 0
    msg.goal.target_pose.pose.orientation.y = 0
    msg.goal.target_pose.pose.orientation.z = 0.999646735844
    msg.goal.target_pose.pose.orientation.w = 0.0265782526858
    sleep(2)
    pub.publish(msg)  # 发布消息
    print("pub")



if __name__ == '__main__':
    main()
