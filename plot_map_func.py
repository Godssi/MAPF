import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt


def plot_map_console(maze):
    for i in range(len(maze)):
        for j in range(len(maze[i])):
            print(maze[i][j], end=" ")
        print()


def plot_map_3d(maze):
    h = len(maze)
    w = len(maze[0])

    fig = plt.figure(figsize=(20, 20))
    ax = fig.add_subplot(projection='3d')
    X = np.arange(0, h, 1)
    Y = np.arange(w, 0, -1)
    X, Y = np.meshgrid(X, Y)
    surf = ax.plot_surface(X, Y, maze, cmap=cm.coolwarm,
                           linewidth=0, antialiased=False)
    plt.axis([-1, w + 1, -1, h + 1])
    fig.colorbar(surf, shrink=0.5, aspect=5)
    plt.rc('font', size=(-(1/9) * h + 21))
    plt.show()


def plot_map_2d(maze):
    h = len(maze)
    w = len(maze[0])

    fig = plt.figure(figsize=(20, 20))
    X = np.arange(0, h, 1)
    Y = np.arange(0, w, 1)
    plt.contourf(X, Y, maze.T, 500, cmap='seismic')
    plt.axis([-1, w + 1, -1, h + 1])
    plt.rc('font', size=(-(1/9) * h + 21))
    plt.colorbar()
    plt.show()


def plot_origin_map_2d(maze, path=None):
    h = len(maze)
    w = len(maze[0])
    plt.rc('font', size=(-(1/9) * h + 21))
    fig = plt.figure(figsize=(20, 20))
    x_idx = []
    y_idx = []
    for i in range(h):
        for j in range(w):
            if maze[i][j] == 1:
                x_idx.append(i)
                y_idx.append(j)
    marker_size = 12100         # default marker size
    if h <= 30:
        f = lambda x: 97/7500 * x ** 5 - 517/375 * x ** 4 + 16139/300 * x ** 3 - 13469/15 * x ** 2 + 4931 * x + 11280
        marker_size = f(h)
    elif h <= 50:
        f = lambda x: -1/100 * x ** 3 + 49/20 * x ** 2 - 371/2 * x + 4770
        marker_size = f(h)
    elif h <= 70:
        f = lambda x: -7/3000 * x ** 3 + 61/100 * x ** 2 - 853/15 * x + 1980
        marker_size = f(h)
    else:
        f = lambda x: -1/750 * x ** 3 + 41/100 * x ** 2 - 1313/30 * x + 1700
        marker_size = f(h)
    plt.scatter(x_idx, y_idx, s=marker_size, marker='s')
    plt.scatter(0, 0, s=marker_size, marker='s')
    plt.text(0, 0, 'Start', verticalalignment='center', horizontalalignment='center')
    plt.scatter(w - 1, h - 1, s=marker_size, marker='s')
    plt.text(w - 1, h - 1, 'End', verticalalignment='center', horizontalalignment='center')

    if path is not None:
        x = np.array(path).T[0]
        y = np.array(path).T[1]
        plt.plot(x, y, 'r', linewidth=(-(1/9) * h + 14))
    plt.axis([-1, w + 1, -1, h + 1])
    plt.show()
