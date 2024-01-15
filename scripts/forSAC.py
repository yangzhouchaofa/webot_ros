import os
import pickle
import math
import numpy as np
import rospy
from time import time, sleep
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import LaserScan
from webots_ros.srv import get_floatRequest, get_float

action = [0, 0]
gps_names = ['gpsFront', 'gpsBack']


class recommand():
    def __init__(self, pickle_name, valuename):
        self.PATH = '/home/ylc/webots_ros/src/webots_ros/SAC1/data%d'
        self.PICKLE = pickle_name
        self.value_name = valuename
        self.value = []
        self.obs = [[3.69, 0.68],[1.88, 1.06],[3.27, 2.02],[1.96, 0.23],[0.73, 1.75]]
        self.done = False
        self.num = 0
        while (os.path.exists(self.PATH%self.num)):
            self.num += 1


    def wait_for_plan(self):
        while (len(rospy.wait_for_message('/move_base/status', GoalStatusArray).status_list) == 0):
            sleep(1)
        while (rospy.wait_for_message('/move_base/status', GoalStatusArray).status_list[0].status != 1):
            sleep(1)

    def receive_gps(self):
        while not self.done:
            value_sub = rospy.wait_for_message('/robot/' + self.value_name + '/values', PointStamped)
            self.value.append([value_sub.point.x, value_sub.point.y, value_sub.point.z, time()])
            status = rospy.wait_for_message('/move_base/status', GoalStatusArray)
            print(time())
            if status.status_list[0].status == 3:
                self.done = True
                print(self.done)
                print(len(self.value))

    def receive_laser(self):
        while not self.done:
            value_sub = rospy.wait_for_message('/robot/rslidar/laser_scan', LaserScan)
            self.value.append([list(value_sub.ranges), time()])
            status = rospy.wait_for_message('/move_base/status', GoalStatusArray)
            print(time())
            if status.status_list[0].status == 3:
                self.done = True
                print(self.done)
                print(len(self.value))

    def receive_motor(self):
        while not self.done:
            value_sub = rospy.wait_for_message('/cmd_vel', Twist)
            self.value.append([value_sub.linear.x,value_sub.angular.z,time()])
            status = rospy.wait_for_message('/move_base/status', GoalStatusArray)
            print(time())
            if status.status_list[0].status == 3:
                self.done = True
                print(self.done)
                print(len(self.value))

    def save(self):
        self.check_and_make(self.PATH % self.num)
        with open(os.path.join(self.PATH % self.num, self.PICKLE), 'wb') as f:
            pickle.dump(self.value, f)

    def save_obs(self):
        self.check_and_make(self.PATH % self.num)
        with open(os.path.join(self.PATH % self.num, 'obs.pkl'), 'wb') as f:
            pickle.dump(self.obs, f)

    def check_and_make(self, dir):
        if not os.path.exists(dir):
            os.makedirs(dir)


def data_for_2Hz(num):
    PATH = '/home/ylc/webots_ros/src/webots_ros/data%d'
    PICKLE0 = 'laser.pkl'
    PICKLE1 = 'gpsfront.pkl'
    PICKLE2 = 'gpsback.pkl'
    PICKLE4 = 'motor.pkl'

    PICKLE = [PICKLE0, PICKLE1, PICKLE2, PICKLE4]
    data = []
    for i in range(len(PICKLE)):
        with open(os.path.join(PATH%num, PICKLE[i]), 'rb') as c:
            data.append(pickle.load(c))
            print(len(data[i]))

    start = max(data[0][0][-1], data[1][0][-1], data[2][0][-1])
    print(data[0][0][-1], data[1][0][-1], data[2][0][-1], start)

    for i in range(len(PICKLE)):
        for j in range(max(len(data[0]), len(data[1]), len(data[2])) - min(len(data[0]), len(data[1]), len(data[2]))):
            if (data[i][0][-1] < start) and ((start - data[i][0][-1]) > 0.05):
                del (data[i][j])

    print(data[0][0][-1], data[1][0][-1], data[2][0][-1])

    for i in range(3):
        data[i] = data[i][::5]

    data_motor = [0] * len(data[1])  # 创建一个数组用来保存查找后的速度
    for i in range(len(data[1])):  # 对于查找后的长度
        actual_time = data[1][i][-1]
        diff_0 = abs(data[3][0][-1] - actual_time)
        data_motor[i] = data[3][0]
        for j in range(len(data[3])):
            diff_j = abs(data[3][j][-1] - actual_time)
            if diff_j < diff_0:
                data_motor[i] = data[3][j]
                diff_0 = diff_j
    data[3] = data_motor

    for i in range(len(PICKLE)):
        for j in range(len(data[i])):
            data[i][j] = data[i][j][:-1]

    for i in range(len(PICKLE)):
        print(len(data[i]))

    return data


def normalize_to_range(value, min, max, newMin, newMax):
    value = float(value)
    min = float(min)
    max = float(max)
    newMin = float(newMin)
    newMax = float(newMax)
    return (newMax - newMin) / (max - min) * (value - max) + newMax


def normalize(value, Min, Max):
    value = float(value)
    Min = float(Min)
    Max = float(Max)
    return round((value - Min) / (Max - Min), 2)


def get_observations(data):
    tarPosition = []
    tarPosition.append(data[2][-1][0])  # x
    tarPosition.append(data[2][-1][2])  # y
    tarPosition.append(-math.atan2(data[1][-1][2] - data[2][-1][2], data[1][-1][0] - data[2][-1][0]))
    observation = []

    noise_laser = np.random.normal(0, 0.01)
    noise_odom = np.random.normal(0, 0.03)
    for i in range(len(data)):
        position = []
        position.append(data[2][i][0])  # x
        position.append(data[2][i][2])  # y
        position.append(-math.atan2(data[1][i][2] - data[2][i][2], data[1][i][0] - data[2][i][0]))
        dx = position[0] - tarPosition[0]
        dy = position[1] - tarPosition[1]
        disReward = math.sqrt(dx * dx + dy * dy)
        laserRange = data[0][i][0]

        # angReward varies from [-pi,pi] relative to tarPosition and left positive / right negtive
        if tarPosition[2] >= 0:
            if position[2] - tarPosition[2] <= -math.pi:
                angReward = position[2] - tarPosition[2] + 2 * math.pi
            else:
                angReward = position[2] - tarPosition[2]
        else:
            if position[2] - tarPosition[2] >= math.pi:
                angReward = position[2] - tarPosition[2] - 2 * math.pi
            else:
                angReward = position[2] - tarPosition[2]
        #       speed   x    y    angle   dx     dy    angle_d   8
        # min   -1.4  -6.2 -4.92  -3.14  -6.2  -4.92   -3.14
        # max    1.4  1.82  1.99   3.14  1.82   1.99    3.14

        CurrentSpeed = (data[3][i][0] + data[3][i][1]) / 2
        observation.append(normalize_to_range(np.clip(position[0] + noise_odom, 0.36, 4.25), 0.36, 4.25, -1, 1))
        observation.append(normalize_to_range(np.clip(position[1] + noise_odom, -0.22, 2.61), -0.22, 2.61, -1, 1))
        observation.append(normalize_to_range(np.clip(position[2], -3.14, 3.14), -3.14, 3.14, -1, 1))
        observation.append(normalize_to_range(np.clip(CurrentSpeed, -1, 1), -1, 1, -1, 1))
        observation.append(normalize_to_range(np.clip(disReward, -2.25, 2.25), -2.25, 2.25, -1, 1))
        observation.append(normalize_to_range(np.clip(angReward, -3.14, 3.14), -3.14, 3.14, -1, 1))
        observation.append(normalize_to_range(np.clip(tarPosition[0], -0.67, 4), -0.67, 4, -1, 1))
        observation.append(normalize_to_range(np.clip(tarPosition[1], 0.2, 2.3), 0.2, 2.3, -1, 1))
        observation.append(normalize_to_range(np.clip(tarPosition[2], -3.14, 3.14), -3.14, 3.14, -1, 1))
        # observation.append(min(self.laserRange))

        # ******************************************************************************
        laser_choose = [0, 32, 360 - 32, 90, 360 - 90, 180 + 43, 180 - 43, 180]
        crash_range = [0.353, 0.456, 0.456, 0.288, 0.288, 0.456, 0.456, 0.353]
        for choose in range(len(laser_choose)):
            laserRange[laser_choose[choose]] -= crash_range[choose]
        for choose in laser_choose:
            observation.append(normalize_to_range(np.clip(laserRange[choose] + noise_laser, 0, 4), 0, 4, 0, 1))
        # ******************************************************************************
        action = [data[3][i][0], data[3][i][1]]
        # reward = get_reward(action)

    return observation

    # def get_reward(action):
    #     reward = 0
    #     deltaDis = disRewardOld - disReward
    #     angleDis = abs(angReward)
    #     flag = 0
    #
    #     reward -= 0.1
    #     if deltaDis > 0 and car_crash() is False:
    #         if (int)(disReward / DistanceRewardInterval) < (int)(disRewardOld / DistanceRewardInterval):
    #             reward += 0.1 * normalize_to_range(np.clip(2.25-disReward,0,2.25),0,2.25,0,1)
    #         if angleDis < math.pi / 2:
    #             reward += 0.1 * normalize_to_range(math.pi / 2 - angleDis, 0, math.pi / 2, 0, 1)
    #
    #     else:
    #         if (int)(disReward / DistanceRewardInterval/2) > (int)(disRewardOld / DistanceRewardInterval/2):
    #             reward += -1
    #
    #
    #     if car_crash() is True:
    #         reward += -5
    #
    #     if disReward <= DistanceThreshold and car_crash() is False:
    #         temp = 0
    #
    #         if abs(normalize_to_range(np.clip(CurrentSpeed, -1, 1), -1, 1, -1, 1)) <= SpeedTheshold \
    #                 and angleDis < RotationThreshold:
    #             temp = 40
    #             if angleDis > 0:
    #                 temp = 40 - 40*normalize(angleDis, 0, RotationThreshold)
    #         reward += temp
    #
    #     if reward == float('-inf') or reward == float('inf'):
    #         reward = 0
    #
    #     if SpeedChange() is True :
    #         reward -= 1
    #
    #     disRewardOld = disReward
    #     return reward


def main():
    # rospy.init_node('forSAC', anonymous=True)
    # while not rospy.is_shutdown():
        # action = get_action()
        # ob = get_observations()
        # print(ob)
    data = data_for_2Hz(1)
    get_observations(data)


if __name__ == '__main__':
    main()
