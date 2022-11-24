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
                            potential_map[k][m] += alpha * np.sqrt((k - i) ** 2 + (m - j) ** 2)
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
    maze = potential_map_generator(maze)
    plot_map_3d(maze)
    print(maze)
