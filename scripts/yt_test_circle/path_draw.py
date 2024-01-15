import numpy as np
import matplotlib.pyplot as plt
from math import *

path_history_path = 'path/path_history'

if __name__ == '__main__':
    path_history = np.load(path_history_path + '.npy')
    ob_x = np.load('path/path_history_obs_x' + '.npy')
    ob_y = np.load('path/path_history_obs_y' + '.npy')
    fig, ax = plt.subplots()
    ax.set_xlim(0,4.6)
    ax.set_ylim(0,3.5)
    plt.plot(path_history[:,0], 2.9 - path_history[:,1])
    plt.scatter(x = ob_x, y = 2.9 - ob_y,s = 2500,marker = '.',c = 'y')
    plt.scatter(x=path_history[-1][0], y=2.9 - path_history[-1][1], s=500, marker='*', c='r')
    plt.show()