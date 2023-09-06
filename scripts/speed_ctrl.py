#!/usr/bin/env python


import rospy
from webots_ros.srv import set_float, set_floatRequest, get_float, get_floatRequest
from geometry_msgs.msg import Twist

Names = ['wheel_lm_motor', 'wheel_rm_motor']
speeds = [0.0, -0.0]


class MOTOR_CONTROL():

    def __init__(self, motorNames):
        self.motorNames = motorNames
        self.vel = [0, 0]

    def init_speed(self):
        for i in range(len(self.motorNames)):
            set_position_client = rospy.ServiceProxy('/robot/' + self.motorNames[i] + '/set_position', set_float)
            set_position_srv = set_floatRequest()
            set_position_srv.value = float("inf")
            if set_position_client(set_position_srv):
                rospy.loginfo("Position set to INFINITY for motor %s.", self.motorNames[i])
            else:
                rospy.logerr("Failed to call service set_position on motor %s.", self.motorNames[i])

            set_velocity_client = rospy.ServiceProxy('/robot/' + self.motorNames[i] + '/set_velocity', set_float)
            set_velocity_srv = set_floatRequest()
            set_velocity_srv.value = 0.0
            if set_velocity_client(set_velocity_srv):
                rospy.loginfo("Velocity set to 0.0 for motor %s.", self.motorNames[i])
            else:
                rospy.logerr("Failed to call service set_velocity on motor %s.", self.motorNames[i])

    def update_speed(self, speeds): # 更新速度
        for i in range(len(self.motorNames)):
            set_velocity_client = rospy.ServiceProxy('/robot/' + self.motorNames[i] + '/set_velocity', set_float)
            set_velocity_req = set_floatRequest()
            set_velocity_req.value = -speeds[i]
            set_velocity_client(set_velocity_req)

            # set_velocity_resp = set_velocity_client(set_velocity_req)
            # if set_velocity_resp:
            #     rospy.loginfo('Speed updated successfully for motor: ' + self.motorNames[i])
            # else:
            #     rospy.logerr('Failed to update speed for motor: ' + self.motorNames[i])

    # def velocity_publisher(self):
    #     pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #     cmd_vel_msg = Twist()
    #     cmd_vel_msg.linear.x = (self.vel[1] + self.vel[0]) * 0.314 / 2 # 设置线性速度 (m/s)
    #     cmd_vel_msg.angular.z = (self.vel[0] - self.vel[1]) * 0.628 / 0.398 # 设置角速度 (rad/s)
    #     pub.publish(cmd_vel_msg) # 发布消息

    def velocity_get_server(self):
        for i in range(len(self.motorNames)):
            rospy.wait_for_service('/robot/' + self.motorNames[i] + '/get_velocity')
            get_vel_service = rospy.ServiceProxy('/robot/' + self.motorNames[i] + '/get_velocity', get_float)
            get_vel_requests = get_floatRequest()
            self.vel[i] = float(str(get_vel_service(get_vel_requests))[7:])


def main():
    rospy.init_node('speed_ctrl', anonymous=True)
    motor = MOTOR_CONTROL(Names)
    motor.init_speed()
    while not rospy.is_shutdown():
        sub = rospy.wait_for_message('/cmd_vel', Twist)
        v = sub.linear.x
        w = sub.angular.z
        speeds[1] = ((v - w * 0.199) / (0.1))
        speeds[0] = ((v + w * 0.199) / (0.1))
        # speeds[1] = v / (0.1)
        # speeds[0] = v / (0.1)
        motor.update_speed(speeds)
        # print(v, w, speeds)


if __name__ == '__main__':
    main()
