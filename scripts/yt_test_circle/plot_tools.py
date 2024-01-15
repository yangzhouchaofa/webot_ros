# -*- coding: utf-8 -*-
"""
Created on Tue Dec 24 15:01:10 2019
@author: Hill
"""

import matplotlib.pyplot as plt
import numpy as np

def plot_reward(reward, dir_name):
    plt.figure(3)
    plt.plot(reward, 'r')
    plt.title('reward')
    plt.grid()
    plt.savefig(dir_name + '/reward.png')


def plot_loss_critic(loss, dir_name):
    plt.figure(4)
    plt.plot()
    plt.plot(loss, 'b')
    plt.grid()
    plt.title('loss_critic')
    plt.xlabel('episode')
    plt.savefig(dir_name + '/c_loss.png')


def plot_loss_actor(loss, dir_name):
    plt.figure(5)
    plt.plot()
    plt.plot(loss, 'b')
    plt.grid()
    plt.title('loss_actor')
    plt.xlabel('episode')
    plt.savefig(dir_name + '/a_loss.png')


def plot_noise(noise, dir_name):
    plt.figure(6)
    plt.plot(noise)
    plt.title('noise')
    plt.grid()
    plt.savefig(dir_name + '/noise.png')


def plot_control_perform(test_signal, process_output, process_input, dir_name):
    plt.figure(1)
    plt.plot(test_signal, 'r', process_output, 'g', process_input, 'b')
    label = ['target', 'output', 'input']
    plt.grid()
    plt.legend(label, loc=0)
    plt.savefig(dir_name + '/perform.png')


def plot_output(outputs, dir_name):
    plt.figure(7)
    plt.plot(outputs)
    plt.title('outputs')
    plt.grid()
    plt.savefig(dir_name + '/output_memory.png')

def plotLearning_PG(scores, filename, x=None, window=5):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    N = len(scores)
    running_avg = np.empty(N)
    for t in range(N):
        running_avg[t] = np.mean(scores[max(0, t - window):(t + 1)])
    if x is None:
        x = [i for i in range(N)]
    ax.set_ylabel('Score')
    ax.set_xlabel('Game')
    plt.plot(x, running_avg)
    plt.savefig('figs/' + filename)
