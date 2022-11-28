import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt
from agent import Agent


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
    plt.axis([-1, h + 1, -1, w + 1])
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
    plt.axis([-1, h + 1, -1, w + 1])
    plt.rc('font', size=(-(1/9) * h + 21))
    plt.colorbar()
    plt.show()


def plot_origin_map_2d(maze, agent):
    h = len(maze)
    w = len(maze[0])
    path = agent.path
    start_node = agent.start
    end_node = agent.goal

    plt.rc('font', size=(-(1/9) * h + 21))
    fig = plt.figure(figsize=(20, 20))
    fix_x_idx = []
    fix_y_idx = []
    mov_x_idx = []
    mov_y_idx = []
    for i in range(h):
        for j in range(w):
            if maze[i][j] == 1:
                fix_x_idx.append(i)
                fix_y_idx.append(j)
            elif maze[i][j] == 2:
                mov_x_idx.append(i)
                mov_y_idx.append(j)
    marker_size = 12100         # default marker size
    map_size = max(h, w)
    if map_size <= 30:
        f = lambda x: 0.0129334 * x ** 5 - 1.37867 * x ** 4 + 53.8 * x ** 3 - 897.934 * x ** 2 + 4931 * x + 11280
        marker_size = f(map_size)
    elif map_size <= 50:
        f = lambda x: -0.01 * x ** 3 + 2.45 * x ** 2 - 185.5 * x + 4770
        marker_size = f(map_size)
    elif map_size <= 70:
        f = lambda x: -7/3000 * x ** 3 + 0.61 * x ** 2 - 56.8667 * x + 1980
        marker_size = f(map_size)
    else:
        f = lambda x: -1/750 * x ** 3 + 0.41 * x ** 2 - 1313/30 * x + 1700
        marker_size = f(map_size)
    plt.scatter(fix_x_idx, fix_y_idx, s=marker_size, marker='s')
    plt.scatter(mov_x_idx, mov_y_idx, s=marker_size, marker='s')
    if start_node is None and end_node is None:
        plt.scatter(0, 0, s=marker_size, marker='s')
        plt.text(0, 0, 'Start', verticalalignment='center', horizontalalignment='center')
        plt.scatter(w - 1, h - 1, s=marker_size, marker='s')
        plt.text(w - 1, h - 1, 'End', verticalalignment='center', horizontalalignment='center')
    else:
        plt.scatter(start_node[0], start_node[1], s=marker_size, marker='s')
        plt.text(start_node[0], start_node[1], 'Start', verticalalignment='center', horizontalalignment='center')
        plt.scatter(end_node[0], end_node[1], s=marker_size, marker='s')
        plt.text(end_node[0], end_node[1], 'End', verticalalignment='center', horizontalalignment='center')

    if path is not None:
        x = np.array(path).T[0]
        y = np.array(path).T[1]
        plt.plot(x, y, 'r', linewidth=(-(1/9) * h + 14))
    plt.axis([-1, h + 1, -1, w + 1])
    plt.show()
