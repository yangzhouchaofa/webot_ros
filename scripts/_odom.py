#!/usr/bin/env python

import rospy
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist


def main():
    rospy.init_node('odom')

    odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    x = 0.0
    y = 0.0
    th = 0.0

    last_time = rospy.Time.now()

    while not rospy.is_shutdown():

        current_time = rospy.Time.now()
        sub = rospy.wait_for_message('/cmd_vel', Twist)
        vx = sub.linear.x
        vth = sub.angular.z
        # compute odometry in a typical way given the velocities of the robot
        dt = (current_time - last_time).to_sec()
        if(dt<10):
            delta_x = (vx * math.cos(th)) * dt
            delta_y = (vx * math.sin(th)) * dt
            delta_th = vth * dt

            x += delta_x
            y += delta_y
            th += delta_th

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, th))

            # first, we'll publish the transform over tf
            odom_trans = TransformStamped()
            odom_trans.header.stamp = current_time
            odom_trans.header.frame_id = "odom"
            odom_trans.child_frame_id = "base_link"

            odom_trans.transform.translation.x = x
            odom_trans.transform.translation.y = y
            odom_trans.transform.translation.z = 0.0
            odom_trans.transform.rotation = odom_quat

            # send the transform
            odom_broadcaster.sendTransformMessage(odom_trans)

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = odom_quat

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = vth

            # publish the message
            odom_pub.publish(odom)

        last_time = current_time

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
