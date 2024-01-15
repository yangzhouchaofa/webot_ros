from environment import Hunter
import os
import pickle
import math
import numpy as np

from utilities import normalize_to_range


class ros_Hunter(Hunter):
    def __init__(self, num):

        self.PATH = '/home/ylc/webots_ros/src/webots_ros/SAC0/data%d'
        self.PICKLE0 = 'laser.pkl'
        self.PICKLE1 = 'gpsfront.pkl'
        self.PICKLE2 = 'gpsback.pkl'
        self.PICKLE4 = 'motor.pkl'
        self.PICKLE = [self.PICKLE0, self.PICKLE1, self.PICKLE2, self.PICKLE4]
        self.data = []
        self.num = num

        for i in range(len(self.PICKLE)):
            with open(os.path.join(self.PATH % self.num, self.PICKLE[i]), 'rb') as c:
                self.data.append(pickle.load(c))

        start = max(self.data[0][0][-1], self.data[1][0][-1], self.data[2][0][-1])

        for i in range(len(self.PICKLE)):
            for j in range(
                    max(len(self.data[0]), len(self.data[1]), len(self.data[2])) - min(len(self.data[0]),
                                                                                       len(self.data[1]),
                                                                                       len(self.data[2]))):
                if (self.data[i][0][-1] < start) and ((start - self.data[i][0][-1]) > 0.05):
                    del (self.data[i][j])

        for i in range(3):
            self.data[i] = self.data[i][::5]

        data_motor = [0] * len(self.data[1])  # 创建一个数组用来保存查找后的速度
        for i in range(len(self.data[1])):  # 对于查找后的长度
            actual_time = self.data[1][i][-1]
            diff_0 = abs(self.data[3][0][-1] - actual_time)
            data_motor[i] = self.data[3][0]
            for j in range(len(self.data[3])):
                diff_j = abs(self.data[3][j][-1] - actual_time)
                if diff_j < diff_0:
                    data_motor[i] = self.data[3][j]
                    diff_0 = diff_j
        self.data[3] = data_motor

        for i in range(len(self.PICKLE)):
            for j in range(len(self.data[i])):
                self.data[i][j] = self.data[i][j][:-1]

        self.tarPosition = []
        self.tarPosition.append(self.data[2][-1][0])  # x
        self.tarPosition.append(self.data[2][-1][2])  # y
        self.tarPosition.append(
            -math.atan2(self.data[1][-1][2] - self.data[2][-1][2], self.data[1][-1][0] - self.data[2][-1][0]))

        self.disReward = float("inf")
        self.disRewardOld = float("inf")
        self.angReward = float("inf")
        self.DistanceRewardInterval = 0.15
        self.DistanceThreshold = 0.10
        self.RotationThreshold = 0.20
        self.CurrentSpeed = 0
        self.SpeedTheshold = 0.1
        self.park_count = 0
        self.crash_count = 0
        self.Speed_list = []
        self.avg_speed = 0
        self.Repeat_reward = []
        self.complete = 0



    def get_observations(self, i):

        observation = []

        noise_laser = np.random.normal(0, 0.01)
        noise_odom = np.random.normal(0, 0.03)

        position = []
        position.append(self.data[2][i][0])  # x
        position.append(self.data[2][i][2])  # y
        position.append(
            -math.atan2(self.data[1][i][2] - self.data[2][i][2], self.data[1][i][0] - self.data[2][i][0]))
        dx = position[0] - self.tarPosition[0]
        dy = position[1] - self.tarPosition[1]
        self.disReward = math.sqrt(dx * dx + dy * dy)
        self.laserRange = self.data[0][i][0]

        # angReward varies from [-pi,pi] relative to tarPosition and left positive / right negtive
        if self.tarPosition[2] >= 0:
            if position[2] - self.tarPosition[2] <= -math.pi:
                self.angReward = position[2] - self.tarPosition[2] + 2 * math.pi
            else:
                self.angReward = position[2] - self.tarPosition[2]
        else:
            if position[2] - self.tarPosition[2] >= math.pi:
                self.angReward = position[2] - self.tarPosition[2] - 2 * math.pi
            else:
                self.angReward = position[2] - self.tarPosition[2]
        #       speed   x    y    angle   dx     dy    angle_d   8
        # min   -1.4  -6.2 -4.92  -3.14  -6.2  -4.92   -3.14
        # max    1.4  1.82  1.99   3.14  1.82   1.99    3.14
        v = self.data[3][i][0]
        w = self.data[3][i][1]
        # print(v,w,-((v + w * 0.199) / (0.1)), -((v - w * 0.199) / (0.1)))
        self.action = [-((v + w * 0.199) / (0.1)), -((v - w * 0.199) / (0.1))]
        self.CurrentSpeed = (self.action[0]/5 + self.action[1]/5) / 2

        observation.append(normalize_to_range(np.clip(position[0] + noise_odom, 0.36, 4.25), 0.36, 4.25, -1, 1))
        observation.append(normalize_to_range(np.clip(position[1] + noise_odom, -0.22, 2.61), -0.22, 2.61, -1, 1))
        observation.append(normalize_to_range(np.clip(position[2], -3.14, 3.14), -3.14, 3.14, -1, 1))
        observation.append(normalize_to_range(np.clip(self.CurrentSpeed, -1, 1), -1, 1, -1, 1))
        observation.append(normalize_to_range(np.clip(self.disReward, -2.25, 2.25), -2.25, 2.25, -1, 1))
        observation.append(normalize_to_range(np.clip(self.angReward, -3.14, 3.14), -3.14, 3.14, -1, 1))
        observation.append(normalize_to_range(np.clip(self.tarPosition[0], -0.67, 4), -0.67, 4, -1, 1))
        observation.append(normalize_to_range(np.clip(self.tarPosition[1], 0.2, 2.3), 0.2, 2.3, -1, 1))
        observation.append(normalize_to_range(np.clip(self.tarPosition[2], -3.14, 3.14), -3.14, 3.14, -1, 1))
        # observation.append(min(self.laserRange))

        # ******************************************************************************
        self.laser_choose = [0, 32, 360 - 32, 90, 360 - 90, 180 + 43, 180 - 43, 180]
        self.crash_range = [0.353, 0.456, 0.456, 0.288, 0.288, 0.456, 0.456, 0.353]
        for choose in range(len(self.laser_choose)):
            self.laserRange[self.laser_choose[choose]] -= self.crash_range[choose]
        for choose in self.laser_choose:
            observation.append(normalize_to_range(np.clip(self.laserRange[choose] + noise_laser, 0, 4), 0, 4, 0, 1))
        # ******************************************************************************

        return observation
