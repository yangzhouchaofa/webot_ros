import os
import pickle
import numpy as np

PATH = '/home/ylc/webots_ros/src/webots_ros/data1'
PICKLE0 = 'laser.pkl'
PICKLE1 = 'gpsfront.pkl'
PICKLE2 = 'gpsback.pkl'
PICKLE4 = 'motor.pkl'

PICKLE = [PICKLE0, PICKLE1, PICKLE2, PICKLE4]
data = []
for i in range(len(PICKLE)):
    with open(os.path.join(PATH, PICKLE[i]), 'rb') as c:
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
