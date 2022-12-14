import numpy as np


def potential_map_generator(maze):
    h = len(maze)
    w = len(maze[0])
    potential_map = np.ones((h, w))
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if maze[i][j] == 1:
                outer_r = 10  # 장애물을 고려해야하는 최대 거리
                inner_r = 2   # 충돌방지 최소 거리
                alpha = 10    # 장애물의 위험도 가중치 계수
                for k in range(i - outer_r, i + outer_r + 1):
                    for m in range(j - outer_r, j + outer_r + 1):
                        r = (k - i) ** 2 + (m - j) ** 2
                        # 안전거리 이내인 경우 inf로 설정
                        if r <= inner_r ** 2 and 0 <= k < h and 0 <= m < w:
                            potential_map[k][m] += alpha
                        # 적정거리까지 장애물을 고려하는 것을 반영
                        elif r <= outer_r ** 2 and 0 <= k < h and 0 <= m < w:
                            r = np.sqrt(r)
                            if r < 1e6:
                                r = 1
                            potential_map[k][m] += (1 / (outer_r - inner_r)) * \
                                                   ((inner_r * outer_r * (alpha - 1)) / r + outer_r - inner_r * alpha)
            elif maze[i][j] == 3:
                outer_r = 5  # 장애물을 고려해야하는 최대 거리
                inner_r = 2  # 충돌방지 최소 거리
                alpha = 4    # wall
                for k in range(i - outer_r, i + outer_r + 1):
                    for m in range(j - outer_r, j + outer_r + 1):
                        r = (k - i) ** 2 + (m - j) ** 2
                        # 안전거리 이내인 경우 inf로 설정
                        if r <= inner_r ** 2 and 0 <= k < h and 0 <= m < w:
                            potential_map[k][m] += alpha
                        # 적정거리까지 장애물을 고려하는 것을 반영
                        elif r <= outer_r ** 2 and 0 <= k < h and 0 <= m < w:
                            r = np.sqrt(r)
                            if r < 1e6:
                                r = 1
                            potential_map[k][m] += (1 / (outer_r - inner_r)) * \
                                                   ((inner_r * outer_r * (alpha - 1)) / r + outer_r - inner_r * alpha)
    return potential_map


def modify_potential_map(maze, potential_map):
    h = len(maze)
    w = len(maze[0])
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if maze[i][j] == 2:
                outer_r = 7  # 장애물을 고려해야하는 최대 거리
                inner_r = 2  # 충돌방지 최소 거리
                alpha = 28  # 사람의 위험도 가중치 계수
                for k in range(i - outer_r, i + outer_r + 1):
                    for m in range(j - outer_r, j + outer_r + 1):
                        r = (k - i) ** 2 + (m - j) ** 2
                        # 안전거리 이내인 경우 inf로 설정
                        if r <= inner_r ** 2 and 0 <= k < h and 0 <= m < w:
                            potential_map[k][m] += alpha
                        # 적정거리까지 장애물을 고려하는 것을 반영
                        elif r <= outer_r ** 2 and 0 <= k < h and 0 <= m < w:
                            r = np.sqrt(r)
                            if r < 1e6:
                                r = 1
                            potential_map[k][m] += (1 / (outer_r - inner_r)) * \
                                                   ((inner_r * outer_r * (alpha - 1)) / r + outer_r - inner_r * alpha)
    return potential_map


def test_maze_gen_1():
    maze = [[3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3],
            [3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3],
            [3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3]]
    return maze


def test_maze_gen_2():
    maze = [[3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3],
            [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 3],
            [3, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 3],
            [3, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
            [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
            [3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3]]
    return maze


def random_maze_gen():
    map_size1 = np.random.randint(30, 50)
    map_size2 = np.random.randint(30, 50)
    # map_size = 100
    new_map = np.zeros((map_size1, map_size2), dtype=np.int64)
    for i in range(map_size1):
        for j in range(map_size2):
            new_map[i, j] = np.random.randint(0, 500) / 495
    # 외부 벽 생성
    for i in range(map_size1):
        new_map[i, 0] = 3
        new_map[i, map_size2 - 1] = 3
    for i in range(map_size2):
        new_map[0, i] = 3
        new_map[map_size1 - 1, i] = 3
    new_map[1:3, 1:3] = 0
    new_map[map_size1 - 2:map_size1 - 4, map_size2 - 2:map_size2 - 4] = 0
    # new_map[0:2, 0:2] = 0
    # new_map[map_size - 1:map_size - 3, map_size - 1:map_size - 3] = 0
    return new_map.tolist()
