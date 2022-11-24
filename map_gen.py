import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt


def potential_map_generator(maze):
    h = len(maze)
    w = len(maze[0])
    potential_map = np.ones((h, w))
    outer_r = 3     # 장애물을 고려해야하는 최대 거리
    inner_r = 1     # 충돌방지 최소 거리
    alpha = 200    # 장애물의 위험도 가중치 계수
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if maze[i][j] == 1:
                for k in range(i - outer_r, i + outer_r + 1):
                    for m in range(j - outer_r, j + outer_r + 1):
                        # 안전거리 이내인 경우 inf로 설정
                        if (k - i) ** 2 + (m - j) ** 2 <= inner_r ** 2 and 0 <= k < h and 0 <= m < w:
                            potential_map[k][m] += alpha
                        # 적정거리까지 장애물을 고려하는 것을 반영
                        if (k - i) ** 2 + (m - j) ** 2 <= outer_r ** 2 and 0 <= k < h and 0 <= m < w:
                            r = np.sqrt((k - i) ** 2 + (m - j) ** 2)
                            if r < 1e6:
                                r = 1
                            potential_map[k][m] += (1 / (outer_r - inner_r)) * (inner_r * outer_r * (alpha - 1) / r + outer_r - inner_r * alpha)
    return potential_map


# def potential_map_generator(maze):
#     h = len(maze)
#     w = len(maze[0])
#     potential_map = np.ones((h, w))
#     outer_r = 4     # 장애물을 고려해야하는 최대 거리
#     inner_r = 1     # 충돌방지 최소 거리
#     alpha = 100    # 장애물의 위험도 가중치 계수
#     for i in range(len(maze)):
#         for j in range(len(maze[0])):
#             if maze[i][j] == 1:
#                 for k in range(i - outer_r, i + outer_r + 1):
#                     for m in range(j - outer_r, j + outer_r + 1):
#                         # 안전거리 이내인 경우 inf로 설정
#                         if (k - i) ** 2 + (m - j) ** 2 <= inner_r ** 2 and 0 <= k < h and 0 <= m < w:
#                             potential_map[k][m] += alpha + 1
#                             # 적정거리까지 장애물을 고려하는 것을 반영
#                         if (k - i) ** 2 + (m - j) ** 2 <= outer_r ** 2 and 0 <= k < h and 0 <= m < w:
#                             r = np.sqrt((k - i) ** 2 + (m - j) ** 2)
#                             potential_map[k][m] += -alpha * (r - inner_r) / (outer_r - inner_r) + alpha + 1
#     return potential_map


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
    fig.colorbar(surf, shrink=0.5, aspect=5)
    plt.rc('font', size=20)
    plt.show()


def plot_map_2d(maze):
    h = len(maze)
    w = len(maze[0])

    fig = plt.figure(figsize=(20, 20))
    X = np.arange(0, h, 1)
    Y = np.arange(w, 0, -1)
    plt.contourf(X, Y, maze, 1000)
    plt.rc('font', size=50)
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
        f = lambda x: 97/7500 * x ** 5 - 517/375 * x ** 4 + 16139/300 * x ** 3 - 13469/15 * x ** 2 + 4931 * x +11280
        marker_size = f(h)
    elif h <= 70:
        f = lambda x: 13/2100000 * x ** 5 - 671/420000 * x ** 4 + 18/128 * x ** 3 - 19399/4200 * x ** 2 - 3071/105 * x +3430
        marker_size = f(h)
    else:
        f = lambda x: -1/750 * x ** 3 + 41/100 * x ** 2 - 1313/30 *x + 1700
        marker_size = f(h)
    plt.scatter(x_idx, y_idx, s=marker_size, marker='s')
    plt.scatter(1, 1, s=marker_size, marker='s')
    plt.text(1, 1, 'Start', verticalalignment='center', horizontalalignment='center')
    plt.scatter(w - 2, h - 2, s=marker_size, marker='s')
    plt.text(w - 2, h - 2, 'End', verticalalignment='center', horizontalalignment='center')

    if path is not None:
        x = np.array(path).T[0]
        y = np.array(path).T[1]
        plt.plot(x, y, 'r', linewidth=(-(1/9) * h + 14))
    plt.axis([-1, w + 1, -1, h + 1])
    plt.show()


def random_maze_gen():
    # map_size = np.random.randint(20, 30)
    map_size = 20
    new_map = np.zeros((map_size, map_size), dtype=np.int64)
    for i in range(map_size):
        for j in range(map_size):
            new_map[i, j] = np.random.randint(0, 50)
            if new_map[i, j] <= 45:
                new_map[i, j] = 0
            else:
                new_map[i, j] = 1
    for i in range(map_size):
        new_map[i, 0] = 1
        new_map[0, i] = 1
        new_map[i, map_size - 1] = 1
        new_map[map_size - 1, i] = 1
    new_map[1:3, 1:3] = 0
    new_map[map_size - 2:map_size - 4, map_size - 2:map_size - 4] = 0
    return new_map.tolist()


if __name__ == '__main__':
    np.set_printoptions(linewidth=np.inf)
    maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 1, 0]]
    maze = random_maze_gen()
    plot_origin_map_2d(maze)
    maze = potential_map_generator(maze)
    plot_map_2d(maze)
    print(maze)
