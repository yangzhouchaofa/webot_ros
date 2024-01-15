import numpy as np
import torch


class ReplayBuffer(object):

    def __init__(self, max_size, state_dim, action_dim):
        self.max_size = int(max_size)
        self.count = 0
        self.size = 0
        self.s = np.zeros((self.max_size, state_dim))
        self.a = np.zeros((self.max_size, action_dim))
        self.r = np.zeros((self.max_size, 1))
        self.s_ = np.zeros((self.max_size, state_dim))
        self.d = np.zeros((self.max_size, 1))

    def sample(self, batch_size):
        index = np.random.choice(self.size, size=batch_size)
        batch_s = self.s[index]
        batch_a = self.a[index]
        batch_r = self.r[index]
        batch_s_ = self.s_[index]
        batch_d = self.d[index]

        return batch_s, batch_a, batch_r, batch_s_, batch_d

    def len(self):
        return self.count

    def add(self, s, a, r, s_, d):
        self.s[self.count] = s
        self.a[self.count] = a
        self.r[self.count] = r
        self.s_[self.count] = s_
        self.d[self.count] = d
        self.count = (self.count + 1) % self.max_size
        self.size = min(self.size + 1, self.max_size)
