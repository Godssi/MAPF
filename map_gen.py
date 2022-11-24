import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt


def potential_map_generator(maze):
    h = len(maze)
    w = len(maze[0])
    potential_map = np.ones((h, w))
    outer_r = 2     # 장애물을 고려해야하는 최대 거리
    inner_r = 0     # 충돌방지 최소 거리
    alpha = 4       # 장애물의 위험도 가중치 계수
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if maze[i][j] == 1:
                for k in range(i - outer_r, i + outer_r + 1):
                    for m in range(j - outer_r, j + outer_r + 1):
                        # 안전거리 이내인 경우 inf로 설정
                        if (k - i) ** 2 + (m - j) ** 2 <= inner_r ** 2 and 0 <= k < h and 0 <= m < w:       
                            potential_map[k][m] += 1000000
                        # 적정거리까지 장애물을 고려하는 것을 반영
                        if (k - i) ** 2 + (m - j) ** 2 <= outer_r ** 2 and 0 <= k < h and 0 <= m < w:
                            potential_map[k][m] += -alpha * (np.sqrt((k - i) ** 2 + (m - j) ** 2) - inner_r)\
                                                   / (outer_r - inner_r) + alpha + 1
    return potential_map


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

    fig = plt.figure(figsize=(20, 20))
    x_idx = []
    y_idx = []
    for i in range(h):
        for j in range(w):
            if maze[i][j] == 1:
                x_idx.append(i)
                y_idx.append(j)
    if 10 <= h <= 13:
        plt.scatter(x_idx, y_idx, s=5300, marker='s')
    elif 13 < h <= 17:
        plt.scatter(x_idx, y_idx, s=4900, marker='s')
    elif 17 < h <= 20:
        plt.scatter(x_idx, y_idx, s=4500, marker='s')

    if path is not None:
        x = np.array(path).T[0]
        y = np.array(path).T[1]
        plt.plot(x, y, 'r', linewidth=20)
    plt.show()


def random_maze_gen():
    map_size = np.random.randint(10, 20)
    new_map = np.zeros((map_size, map_size), dtype=np.int64)
    for i in range(map_size):
        for j in range(map_size):
            new_map[i, j] = np.random.randint(0, 6)
            if new_map[i, j] <= 4:
                new_map[i, j] = 0
            else:
                new_map[i, j] = 1
    new_map[0, 0] = 0
    new_map[map_size - 1, map_size - 1] = 0
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
    print(maze)
    maze = potential_map_generator(maze)
    print(maze)
