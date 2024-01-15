import os
import torch
from torch.utils.tensorboard import SummaryWriter
import math
import random
import numpy as np
import matplotlib.pyplot as plt

class LogUtil:
    def __init__(self, comment=""):
        self.writer = SummaryWriter(comment=comment)

        self.path_checkpoints = os.path.join("models", os.path.split(self.writer.file_writer.event_writer._logdir)[-1])
        if not os.path.exists(self.path_checkpoints):
            os.makedirs(self.path_checkpoints)

        self.q1_path = os.path.join(self.path_checkpoints, "critic_q1")
        self.q2_path = os.path.join(self.path_checkpoints, "critic_q2")
        self.q1_target_path = os.path.join(self.path_checkpoints, "critic_q1_target")
        self.q2_target_path = os.path.join(self.path_checkpoints, "critic_q2_target")
        self.actor_path = os.path.join(self.path_checkpoints, "actor")
        self.temp_path = os.path.join(self.path_checkpoints, "alpha")
        self.memory_path = os.path.join(self.path_checkpoints, "memory.p")

    def loss(self, q1_loss, q2_loss, pi_loss, alpha_loss, alpha, step):
        self.writer.add_scalar("loss/q1", q1_loss, step)
        self.writer.add_scalar("loss/q2", q2_loss, step)
        self.writer.add_scalar("loss/pi", pi_loss, step)
        self.writer.add_scalar("loss/alpha", alpha_loss, step)
        self.writer.add_scalar("param/alpha", alpha, step)

    def reward(self, episode, reward, type):
        if type == "train":
            self.writer.add_scalar("reward/train", reward, episode)
        elif type == "eval":
            self.writer.add_scalar("reward/eval", reward, episode)

    def save_checkpoints(self, agent, memory):
        torch.save(agent.q1.state_dict(), self.q1_path)
        torch.save(agent.q2.state_dict(), self.q2_path)
        torch.save(agent.q1_target.state_dict(), self.q1_target_path)
        torch.save(agent.q2_target.state_dict(), self.q2_target_path)
        torch.save(agent.policy.state_dict(), self.actor_path)
        torch.save(agent.alpha, self.temp_path)

    def load_checkpoints(self, agent, path):
        agent.q1.load_state_dict(torch.load(os.path.join(path, "critic_q1")))
        agent.q2.load_state_dict(torch.load(os.path.join(path, "critic_q2")))
        agent.q1_target.load_state_dict(torch.load(os.path.join(path, "critic_q1_target")))
        agent.q2_target.load_state_dict(torch.load(os.path.join(path, "critic_q2_target")))
        agent.policy.load_state_dict(torch.load(os.path.join(path, "actor")))


def test_agent(agent, env, render=False):
    state = env.random_initialization()
    total_reward = 0
    steps = 0
    done = False
    while not done:
        if render: env.render()
        action = agent.sample(state)
        next_state, reward, done, _ = env.step(action)

        total_reward += reward
        steps += 1
        state = next_state

    env.close()

    return total_reward, steps


def load_checkpoint(path):
    raise NotImplementedError


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
    return round((value - Min)/(Max - Min),2)


def calcu_reward(env, new_goal, state):
    goal_x, goal_z, goal_ang = new_goal[0], new_goal[1], new_goal[2]
    x, z, ang = state[0], state[1], state[2]
    ds = math.sqrt((goal_x - x) ** 2 + (goal_z - z) ** 2) * 5.31
    da = 9.86 * (abs(goal_ang) - abs(ang)) ** 2
    reward = 0 if (ds < env.DistanceThreshold and da < env.RotationThreshold and env.CurrentSpeed < env.SpeedTheshold) else -1
    return reward

def generate_goals(i, episode_cache, sample_num, sample_range = 200):
    '''
    Input: current steps, current episode transition's cache, sample number
    Return: new goals sets
    notice here only "future" sample policy
    '''
    end = (i+sample_range) if i+sample_range < len(episode_cache) else len(episode_cache)
    epi_to_go = episode_cache[i:end]
    if len(epi_to_go) < sample_num:
        sample_trans = epi_to_go
    else:
        sample_trans = random.sample(epi_to_go, sample_num)
    return [np.array(trans[3][:3]) for trans in sample_trans]

def gene_new_sas(new_goals, transition):
    state, new_state = transition[0][:3], transition[3][:3]
    action = transition[1]
    state = np.concatenate((state, new_goals))
    new_state = np.concatenate((new_state, new_goals))
    return state, action, new_state

def plotLearning_PG(scores, filename, x=None, window=5):
    fig=plt.figure()
    ax=fig.add_subplot(111)
    N = len(scores)
    running_avg = np.empty(N)
    for t in range(N):
        running_avg[t] = np.mean(scores[max(0, t-window):(t+1)])
    if x is None:
        x = [i for i in range(N)]
    ax.set_ylabel('Score')
    ax.set_xlabel('Game')
    plt.plot(x, running_avg)
    plt.savefig('figs/'+filename)
