import os

import rospy

import SAC_runner

"""
large world test_v3.wbt and small world smallworld.wbt
should change the target position and normalize range and initialize range
reduce steptime/gamma/steps
crash_range and test

改变世界后，需要改变目标位置，标准化范围和初始化范围
地图变小后步长相应减小，gamma减小，timestep减小，自己调整
大车与小车比例2：1，地图缩放2：1，各种条件需要缩放2倍
修改碰撞范围并测试
"""

STEPS_PER_EPISODE = 200
EPISODE_LIMIT = 500
score_history_path = 'score/score_history'                   #############change
score_history_continue_path = 'score/score_history_continue' ##############change
fig_path = 'figs/tar_position.png'  #############change
path_history_path = 'path/path_history'                      #############change
train = 'train'

def create_path(path):
    try:
        if not os.path.exists(path):
            os.makedirs(path)
    except OSError:
        print("Creation of the directory %s failed" % path)
    else:
        print("Successfully created the directory %s " % path)

if __name__ == '__main__':
    rospy.init_node('yt_webots', anonymous=True)
    print("Hello")
    create_path("model/sac1/")           #############change
    create_path("./figs/")
    create_path("./score/")
    create_path("./path/")

    # pass a path to load the pretrained models, and pass "" for training from scratch
    save_path = 'model/sac1/'  #############change
    load_path = 'model/sac1/'           #############change
    train = 'test'              #############change   train   train_continue    test

    if train == 'train':
        SAC_runner.train(save_path)
    elif train == 'train_continue':
        SAC_runner.train_continue(load_path)
    elif train == 'test':
        SAC_runner.test(load_path)
    else:
        print('train mode error')