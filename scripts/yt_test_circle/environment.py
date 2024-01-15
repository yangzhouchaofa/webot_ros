import random
import time

import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal, MoveBaseActionResult
import math
import os
import pickle

import rospy
import transforms3d as tf3
from scipy.spatial.transform import Rotation as R
from tf2_msgs.msg import TFMessage
from yt.msg import lidar32
import tf
from tf import TransformListener

from main import train
from controller import Robot, Supervisor
import numpy as np
from gym.spaces import Box, Discrete
from deepbots.supervisor.controllers.robot_supervisor import RobotSupervisor
from utilities import normalize_to_range, normalize
from main import STEPS_PER_EPISODE

CAR_LENGTH = [-0.20, 3.45]
CAR_WIDTH = [-0.6, 1.4]
WORLD_LENGTH = [-0.73, 3,9]
WORLD_WIDTH = [-1.08, 1.92]

class Hunter(RobotSupervisor):

    def __init__(self):
        self.timestep = 1000
        self.observation_space = 14
        self.action_space = Box(low=np.array([-1, -1]), high=np.array([1, 1]), dtype=np.float64)
        self.DistanceRewardInterval = 0.15
        self.DistanceThreshold = 0.1
        self.RotationThreshold = 0.20
        self.CurrentSpeed = 0
        self.w = 0
        self.w_list = []
        self.SpeedTheshold = 1
        self.park_count = 0
        self.crash_count = 0
        self.Speed_list = []
        self.avg_speed = 0
        self.Repeat_reward = []
        self.complete = 0

        self.env_num = 1  # the env number of VectorEnv is greater than 1
        self.env_name = "yt"  # the name of this env.
        self.max_step = STEPS_PER_EPISODE  # the max step of each episode
        self.state_dim = self.observation_space  # feature number of state
        self.action_dim = self.action_space.shape[0]  # feature number of action
        self.if_discrete = False  # discrete action or continuous action
        self.target_return = -200  # episode return is between (-1600, 0)

        print(self.timestep)
        self.laserRange = []
        self.position = []
        self.positionOld = []
        self.disReward = float("inf")
        self.disRewardOld = float("inf")
        self.angReward = float("inf")

        self.front_ds = 0.6  # front safe distance of car
        self.side_ds = 0.4  # beside safe distance of car
        self.behide_ds = 0.6  # back safe distance of car
        self.rec_degree = (round(math.atan(self.side_ds / self.behide_ds) * 180 / math.pi),
                           180 - round(math.atan(self.side_ds / self.behide_ds) * 180 / math.pi) - round(
                               math.atan(self.side_ds / self.front_ds) * 180 / math.pi),
                           round(math.atan(
                               self.side_ds / self.front_ds) * 180 / math.pi))  # use rectangle shape around the car to avoid obstacle

        self.steps = 0
        self.steps_threshold = 6000

        # self.tarPosition = [-0.966, -1.52, -1.57]  # x, z, angle(-3.14,3.14)
        self.tarPosition = [1, 0, 3.14]  # x, z, angle(-3.14,3.14)
        self.wheel_front = []

        self.stepsPerEpisode = 10000  # Max number of steps per episode
        self.episodeScore = 0  # Score accumulated during an episode
        self.episodeScoreList = []  # A list to save all the episode scores, used to check if task is solved

    def motorIn(self, x, y, rot):  # left+ right-
        result = 0
        if x < CAR_LENGTH[0] or x > CAR_LENGTH[1] or y < CAR_WIDTH[0] or y > CAR_WIDTH[1]:
            result = 1
            rospy.loginfo("超出范围！")
        pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        msg = MoveBaseActionGoal()
        msg.goal.target_pose.header.frame_id = "map"
        msg.goal.target_pose.pose.position.x = x
        msg.goal.target_pose.pose.position.y = y
        msg.goal.target_pose.pose.position.z = 0
        msg.goal.target_pose.pose.orientation.x = rot[0]
        msg.goal.target_pose.pose.orientation.y = rot[1]
        msg.goal.target_pose.pose.orientation.z = rot[2]
        msg.goal.target_pose.pose.orientation.w = rot[3]

        while result != 1:
            pub.publish(msg)
            listener = TransformListener()
            listener.waitForTransform("/base_footprint", "/map", rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform('/base_footprint', '/map',
                                                    listener.getLatestCommonTime('/map', '/base_footprint'))
            print("the goal is: ", x, y, "the actual position: ", trans[0], trans[1])
            if abs(trans[0] - x) < 0.2 and abs(trans[1] - y) < 0.2:
                result = 1
                rospy.loginfo("规划成功！")



    # def motorIn(self, w = 0, v = 0):  # left+ right-
    #     pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #     cmd_vel_msg = Twist()
    #     cmd_vel_msg.linear.x = v  # 设置线性速度 (m/s)
    #     cmd_vel_msg.angular.z = w  # 设置角速度 (rad/s)
    #     pub.publish(cmd_vel_msg)  # 发布消息
    #     time.sleep(1)
    #     cmd_vel_msg.linear.x = 0  # 设置线性速度 (m/s)
    #     cmd_vel_msg.angular.z = 0  # 设置角速度 (rad/s)
    #     pub.publish(cmd_vel_msg)  # 发布消息



    def SpeedChange(self):
        self.Speed_list.append(self.CurrentSpeed)
        if len(self.Speed_list) <= 1:
            return False
        elif self.Speed_list[-2] * self.Speed_list[-1] < -0.1:
            return True
        else:
            return False

    @property
    def get_observations(self):

        observation = []
        self.position = []
        self.laserRange = list(rospy.wait_for_message("/yt_lidar32", lidar32).ranges)
        listener = TransformListener()
        # 等待坐标变换树准备就绪
        listener.waitForTransform("/base_footprint", "/map", rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('/base_footprint', '/map',
                                                        listener.getLatestCommonTime('/map', '/base_footprint'))
        euler = tf.transformations.euler_from_quaternion(rot)
        gpsx = trans[0]
        gpsy = - trans[1]
        gpsth = euler[2]
        if train == 'train':
            noise_laser = np.random.normal(0, 0.01)
            noise_odom = np.random.normal(0, 0.03)
        elif train == 'test':
            noise_laser = 0
            noise_odom = 0

        self.position.append(gpsx)  # x
        self.position.append(gpsy)  # y
        self.position.append(gpsth)

        dx = self.position[0] - self.tarPosition[0]
        dy = self.position[1] - self.tarPosition[1]
        self.disReward = math.sqrt(dx * dx + dy * dy)

        # angReward varies from [-pi,pi] relative to tarPosition and left positive / right negtive
        if self.tarPosition[2] >= 0:
            if self.position[2] - self.tarPosition[2] <= -math.pi:
                self.angReward = self.position[2] - self.tarPosition[2] + 2 * math.pi
            else:
                self.angReward = self.position[2] - self.tarPosition[2]
        else:
            if self.position[2] - self.tarPosition[2] >= math.pi:
                self.angReward = self.position[2] - self.tarPosition[2] - 2 * math.pi
            else:
                self.angReward = self.position[2] - self.tarPosition[2]
        #       speed   x    y    angle   dx     dy    angle_d   8
        # min   -1.4  -6.2 -4.92  -3.14  -6.2  -4.92   -3.14
        # max    1.4  1.82  1.99   3.14  1.82   1.99    3.14
        # 当前位置：
        observation.append(normalize_to_range(np.clip(self.position[0] + noise_odom, WORLD_LENGTH[0], WORLD_LENGTH[1]), WORLD_LENGTH[0], WORLD_LENGTH[1], -1, 1))
        observation.append(normalize_to_range(np.clip(self.position[1] + noise_odom, WORLD_WIDTH[0], WORLD_WIDTH[1]), WORLD_WIDTH[0], WORLD_WIDTH[1], -1, 1))
        observation.append(normalize_to_range(np.clip(self.position[2], -3.14, 3.14), -3.14, 3.14, -1, 1))
        observation.append(normalize_to_range(np.clip(self.CurrentSpeed, -0.5, 0.5), -0.5, 0.5, -1, 1))
        observation.append(normalize_to_range(np.clip(self.w, -12, 12), -12, 12, -1, 1))
        observation.append(normalize_to_range(np.clip(self.disReward, 0, 4.8), 0, 4.8, -1, 1))
        observation.append(normalize_to_range(np.clip(self.angReward, -3.14, 3.14), -3.14, 3.14, -1, 1))
        observation.append(normalize_to_range(np.clip(self.tarPosition[0], WORLD_LENGTH[0], WORLD_LENGTH[1]), WORLD_LENGTH[0], WORLD_LENGTH[1], -1, 1))
        observation.append(normalize_to_range(np.clip(self.tarPosition[1], WORLD_WIDTH[0], WORLD_WIDTH[1]), WORLD_WIDTH[0], WORLD_WIDTH[1], -1, 1))
        observation.append(normalize_to_range(np.clip(self.tarPosition[2], -3.14, 3.14), -3.14, 3.14, -1, 1))

        self.laser_choose = range(0,36)
        self.crash_range = [0.353, 0.358, 0.375, 0.408, 0.456, 0.376, 0.332, 0.306, 0.292,
                            0.288, 0.292, 0.306, 0.332, 0.376, 0.456, 0.408, 0.375, 0.358,
                            0.353, 0.358, 0.375, 0.408, 0.456, 0.376, 0.332, 0.306, 0.292,
                            0.288, 0.292, 0.306, 0.332, 0.376, 0.456, 0.408, 0.375, 0.358]

        self.laserx = []
        self.lasery = []

        for choose in range(len(self.laser_choose)):
            x = self.position[0] + self.laserRange[self.laser_choose[choose]] * math.cos( (self.laser_choose[choose] * 31.4) / 180 + self.position[2])
            y = self.position[1] + self.laserRange[self.laser_choose[choose]] * math.sin( (self.laser_choose[choose] * 31.4) / 180 + self.position[2])
            self.laserx.append(x)
            self.lasery.append( - y)
            self.laserRange[self.laser_choose[choose]] -= self.crash_range[choose]
        for i in range(len(self.laserx)):
            observation.append(normalize_to_range(np.clip(self.laserx[i] + noise_laser, WORLD_LENGTH[0], WORLD_LENGTH[1]), WORLD_LENGTH[0], WORLD_LENGTH[1], -1, 1))
            observation.append(normalize_to_range(np.clip(self.lasery[i] + noise_laser, WORLD_WIDTH[0], WORLD_WIDTH[1]), WORLD_WIDTH[0], WORLD_WIDTH[1], -1, 1))

        return observation

    def apply_action(self, action, move):
        listener = TransformListener()
        listener.waitForTransform("/base_footprint", "/map", rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('/base_footprint', '/map',
                                                listener.getLatestCommonTime('/map', '/base_footprint'))
        euler = tf.transformations.euler_from_quaternion(rot)
        tar_x = np.clip(trans[0] + action[0], CAR_LENGTH[0], CAR_LENGTH[1])
        tar_y = np.clip(trans[1] - action[1], CAR_WIDTH[0], CAR_WIDTH[1])

        self.motorIn(tar_x, tar_y, rot)

        # th = euler[2]
        # th_1 = math.atan2(action[1], action[0])
        # if move == 1:
        #     if abs(th_1 - th) > 1.57:
        #         th_1 += 3.14
        #         self.flag = 1
        #     w = (th_1 - th) / 1
        #     self.w = w
        #     self.motorIn(w = w)
        # elif move == 0:
        #     if self.flag == 0:
        #         v = math.sqrt((action[0]) * (action[0]) + (action[1]) * (action[1])) / 1
        #     else:
        #         v = -math.sqrt((action[0]) * (action[0]) + (action[1]) * (action[1])) / 1
        #     if v > 0.5:
        #         v = 0.5
        #     elif v < -0.5:
        #         v = -0.5
        #     self.CurrentSpeed = v
        #     print("v = ", v)
        #     self.motorIn(v = v)


    def step(self, action):
        self.flag = 0
        action[0] = normalize_to_range(action[0], -1, 1, -0.5, 0.5)
        action[1] = normalize_to_range(action[1], -1, 1, -0.5, 0.5)
        self.apply_action(action, 1)
        a = self.get_reward()
        return (
            self.get_observations,
            a,
            self.is_done(),
            self.get_info(),
        )


    def get_reward(self):
        reward = 0
        deltaDis = self.disRewardOld - self.disReward
        angleDis = abs(self.angReward)
        flag = 0

        reward -= 0.1

        if deltaDis > 0 and self.car_crash() is False:
            if (int)(self.disReward / self.DistanceRewardInterval) < (int)(
                    self.disRewardOld / self.DistanceRewardInterval):
                reward += 0.1 * normalize_to_range(np.clip(4.8 - self.disReward, 0, 4.8), 0, 4.8, 0, 1)

        if self.disReward > self.disRewardOld:
                # reward += 1 * (4.8 - self.disReward) * (self.disRewardOld - self.disReward)
                reward += 2 * (self.disRewardOld - self.disReward)
        else:
                # reward += 0.5 * (4.8 - self.disReward) * (self.disRewardOld - self.disReward)
                reward += 1 * (self.disRewardOld - self.disReward)

        if self.car_crash() is True:
            # reward += -10 * self.disReward
            reward += -10

        if self.disReward <= self.DistanceThreshold and self.car_crash() is False:
            reward += 40
        # must add this or the gradient is 'inf', then the loss is 'nan'
        if reward == float('-inf') or reward == float('inf'):
            reward = 0

        if self.SpeedChange() is True:
            reward -= 1

        self.disRewardOld = self.disReward
        # self.dyOld = self.dy
        return reward

    def car_crash(self):
        for i in range(len(self.laser_choose)):
            if self.laserRange[self.laser_choose[i]] < 0.02:
                # print('laseer_range:',self.laserRange[self.laser_choose[i]],'laser_choose:',self.laser_choose[i])
                return True
        return False

    def is_done(self):
        if self.car_crash() is True:
            self.complete = 0
            return False
        elif self.disReward <= self.DistanceThreshold: #and \
                # abs(normalize_to_range(np.clip(self.CurrentSpeed, -0.5, 0.5), -0.5, 0.5, -1, 1)) <= self.SpeedTheshold:  # math.pi / 2
            self.complete = 1
            return True
        else:
            self.complete = 0
            return False

    def solved(self):
        if len(self.episodeScoreList) > 500:  # Over 500 trials thus far
            if np.mean(self.episodeScoreList[-500:]) > 120.0:  # Last 500 episode scores average value
                return True
        return False

    def get_default_observation(self):
        Obs = [0.0 for _ in range(self.observation_space)]
        return Obs

    def render(self, mode='human'):
        print("render() is not used")

    def get_info(self):
        pass

    def random_initialization(self, rb_node=None, Radius=None, tar=None, next=None):
        if tar != None:
            self.tarPosition = tar
        else:
            self.tarPosition[0] = random.uniform(CAR_LENGTH[0], CAR_LENGTH[1])
            self.tarPosition[1] = random.uniform(CAR_WIDTH[0], CAR_WIDTH[1])
            self.tarPosition[2] = random.uniform(-3.14,3.14)

        if rb_node == None:
            rb_node = self.getFromDef("yt")

        else:
            rb_node = rb_node
        self.reset()
        self.motorIn(0, 0)

        # if Radius != None:
        #     x = Radius[0]
        #     z = Radius[1]
        # else:
        x = random.uniform(max(self.tarPosition[0] - next, CAR_LENGTH[0]), min(self.tarPosition[0] + next, CAR_LENGTH[1]))
        z = random.uniform(max(self.tarPosition[1] - next, CAR_WIDTH[0]), min(self.tarPosition[1] + next, CAR_WIDTH[1]))

        y = 0.0912155
        self.rand_rotation_y = R.from_euler('z', random.uniform(0, 360),
                                            degrees=True)  # euler to mat return A matrix,which only use random z axes.

        x_ob = -1
        y_ob = -1
        self.obs_x = []
        self.obs_y = []
        for i in range(5):
            while ((x_ob == -1) or (d2 <= 1) or (d1 <= 1)):
                x_ob = random.uniform(WORLD_LENGTH[0]+0.2, WORLD_LENGTH[1]-0.2)
                y_ob = random.uniform(WORLD_WIDTH[0]+0.2, WORLD_WIDTH[1]-0.2)
                d1 = (x_ob - x) * (x_ob - x) + (y_ob - y) * (y_ob - y)
                d2 = (x_ob - self.tarPosition[0]) * (x_ob - self.tarPosition[0]) + (y_ob - self.tarPosition[1]) * (
                            y_ob - self.tarPosition[1])
            self.obs_x.append(x_ob)
            self.obs_y.append(y_ob)

            ob = self.getFromDef("ob%d" % i)
            ob_position = ob.getField("translation")
            ob_position.setSFVec3f([x_ob, 0.0912155, y_ob])
            x_ob = -1
            y_ob = -1

        ob = self.getFromDef("tar")
        ob_position = ob.getField("translation")
        ob_position.setSFVec3f([self.tarPosition[0], 0.0, self.tarPosition[1]])

        INITIAL = [x, y, z]
        trans_field = rb_node.getField("translation")
        trans_field.setSFVec3f(INITIAL)

        rotation_field = rb_node.getField("rotation")
        quaternion = [0.706522, -0.707691, 0, 0]
        trans_mat = tf3.quaternions.quat2mat(quaternion)

        trans_mat = self.rand_rotation_y.apply(trans_mat)
        quaternion = tf3.quaternions.mat2quat(trans_mat)
        # quaternion to axis angle
        angle = 2 * math.acos(quaternion[0])
        x = quaternion[1] / math.sqrt(1 - quaternion[0] * quaternion[0])
        y = quaternion[2] / math.sqrt(1 - quaternion[0] * quaternion[0])
        z = quaternion[3] / math.sqrt(1 - quaternion[0] * quaternion[0])
        axis_angle = [x, y, z, angle]
        rotation_field.setSFRotation(axis_angle)
        Robot.step(self, self.timestep)  # 更新环境
        return self.get_observations

    def initialization(self, rb_node=None, x=1, z=0, rotation=200, Radius=None, tar=None, next=None):
        if rb_node == None:
            rb_node = self.getFromDef("yt")

        else:
            rb_node = rb_node
        self.reset()
        self.motorIn(0, 0)
        if len(self.obs) != 0:
            for i in range(len(self.obs[0])):
                ob = self.getFromDef("ob%d" % i)
                ob_position = ob.getField("translation")
                ob_position.setSFVec3f([self.obs[0][i][0], 0.0912155, self.obs[0][i][1]])

        ob = self.getFromDef("tar")
        ob_position = ob.getField("translation")
        ob_position.setSFVec3f([self.tarPosition[0], 0.0, self.tarPosition[1]])

        y = 0.2
        self.rand_rotation_y = R.from_euler('z', rotation,
                                            degrees=True)  # euler to mat return A matrix,which only use random z axes.
        INITIAL = [x, y, z]
        trans_field = rb_node.getField("translation")
        trans_field.setSFVec3f(INITIAL)  #

        rotation_field = rb_node.getField("rotation")
        quaternion = [0.707, -0.707, 0, 0]
        trans_mat = tf3.quaternions.quat2mat(quaternion)

        trans_mat = self.rand_rotation_y.apply(trans_mat)
        quaternion = tf3.quaternions.mat2quat(trans_mat)
        # quaternion to axis angle
        angle = 2 * math.acos(quaternion[0])
        x = quaternion[1] / math.sqrt(1 - quaternion[0] * quaternion[0])
        y = quaternion[2] / math.sqrt(1 - quaternion[0] * quaternion[0])
        z = quaternion[3] / math.sqrt(1 - quaternion[0] * quaternion[0])
        axis_angle = [x, y, z, angle]
        rotation_field.setSFRotation(axis_angle)
        Robot.step(self, self.timestep)  # 更新环境
        return self.get_observations

    def test(self, num):
        self.PATH = '/home/ylc/webots_ros/src/webots_ros/SAC1/data%d'
        self.PICKLE0 = 'laser.pkl'
        self.PICKLE1 = 'gpsfront.pkl'
        self.PICKLE2 = 'gpsback.pkl'
        self.PICKLE3 = 'motor.pkl'
        self.PICKLE4 = 'obs.pkl'
        self.PICKLE = [self.PICKLE0, self.PICKLE1, self.PICKLE2, self.PICKLE3]
        self.obs = []
        self.data = []
        self.num = num

        # 记录障碍物的位置
        with open(os.path.join(self.PATH % self.num, self.PICKLE4), 'rb') as c:
            self.obs.append(pickle.load(c))
        # 记录雷达、gps的值
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
            self.data[i] = self.data[i][::10]

        # 对齐motor数据的时间戳
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

        # 去掉时间戳
        for i in range(len(self.PICKLE)):
            for j in range(len(self.data[i])):
                self.data[i][j] = self.data[i][j][:-1]

        for i in range(5):
            self.data[3].append([0.0, 0.0])
            self.data[2].append(self.data[2][-1])
            self.data[1].append(self.data[1][-1])
            self.data[0].append(self.data[0][-1])

        x = (self.data[1][0][0] + self.data[2][0][0]) / 2
        y = (self.data[1][0][2] + self.data[2][0][2]) / 2
        rotation = (math.atan2(self.data[2][0][2] - self.data[1][0][2],
                               self.data[2][0][0] - self.data[1][0][0])) * 180 / math.pi

        self.initialization(x=x, z=y, rotation=rotation, tar=self.tarPosition, )
        self.test_get_observations(0)
        self.disRewardOld = self.disReward
        for i in range(len(self.data[0])):
            self.test_get_observations(i)
            action = self.action
            self.step(action)

        gpsBack = self.gps_back.getValues()
        gpsFront = self.gps_front.getValues()
        self.tarPosition = []
        self.tarPosition.append((gpsBack[0] + gpsFront[0]) / 2)  # x
        self.tarPosition.append((gpsBack[2] + gpsFront[2]) / 2)  # y
        self.tarPosition.append(-math.atan2(gpsFront[2] - gpsBack[2], gpsFront[0] - gpsBack[0]))


    def test_get_observations(self, i):

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
        self.action = [-((v + w * 0.199) / (0.1)) / 5, -((v - w * 0.199) / (0.1)) / 5]
        self.CurrentSpeed = (self.action[0] + self.action[1]) / 2

        observation.append(normalize_to_range(np.clip(position[0] + noise_odom, 0.36, 4.25), 0.36, 4.25, -1, 1))
        observation.append(normalize_to_range(np.clip(position[1] + noise_odom, -0.22, 2.61), -0.22, 2.61, -1, 1))
        observation.append(normalize_to_range(np.clip(position[2], -3.14, 3.14), -3.14, 3.14, -1, 1))
        observation.append(normalize_to_range(np.clip(self.CurrentSpeed, -0.5, 0.5), -0.5, 0.5, -1, 1))
        observation.append(normalize_to_range(np.clip(self.w, -12, 12), -12, 12, -1, 1))
        observation.append(normalize_to_range(np.clip(self.disReward, 0, 4.8), 0, 4.8, -1, 1))
        observation.append(normalize_to_range(np.clip(self.angReward, -3.14, 3.14), -3.14, 3.14, -1, 1))
        observation.append(normalize_to_range(np.clip(self.tarPosition[0], 0.36, 4.25), 0.36, 4.25, -1, 1))
        observation.append(normalize_to_range(np.clip(self.tarPosition[1], -0.22, 2.61), -0.22, 2.61, -1, 1))
        observation.append(normalize_to_range(np.clip(self.tarPosition[2], -3.14, 3.14), -3.14, 3.14, -1, 1))
        # observation.append(min(self.laserRange))

        # ******************************************************************************
        self.laser_choose = [0, 10, 20, 30, 40, 50, 60, 70, 80,
                             90, 100, 110, 120, 130, 140, 150, 160, 170,
                             180, 190, 200, 210, 220, 230, 240, 250, 260,
                             270, 280, 290, 300, 310, 320, 330, 340, 350]
        self.crash_range = [0.353, 0.358, 0.375, 0.408, 0.456, 0.376, 0.332, 0.306, 0.292,
                            0.288, 0.292, 0.306, 0.332, 0.376, 0.456, 0.408, 0.375, 0.358,
                            0.353, 0.358, 0.375, 0.408, 0.456, 0.376, 0.332, 0.306, 0.292,
                            0.288, 0.292, 0.306, 0.332, 0.376, 0.456, 0.408, 0.375, 0.358]
        for choose in range(len(self.laser_choose)):
            self.laserRange[self.laser_choose[choose]] -= self.crash_range[choose]
        for choose in self.laser_choose:
            observation.append(normalize_to_range(np.clip(self.laserRange[choose] + noise_laser, 0, 4), 0, 4, 0, 1))
        # ******************************************************************************

        return observation
