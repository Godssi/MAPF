import numpy as np


def potential_map_generator(maze):
    h = len(maze)
    w = len(maze[0])
    potential_map = np.ones((h, w))
    outer_r = 10     # 장애물을 고려해야하는 최대 거리
    inner_r = 2      # 충돌방지 최소 거리
    alpha = 300      # 장애물의 위험도 가중치 계수
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if maze[i][j] == 1:
                for k in range(i - outer_r, i + outer_r + 1):
                    for m in range(j - outer_r, j + outer_r + 1):
                        r = (k - i) ** 2 + (m - j) ** 2
                        # 안전거리 이내인 경우 inf로 설정
                        if r <= inner_r ** 2 and 0 <= k < h and 0 <= m < w:
                            potential_map[k][m] += alpha + 3000
                        # 적정거리까지 장애물을 고려하는 것을 반영
                        elif r <= outer_r ** 2 and 0 <= k < h and 0 <= m < w:
                            r = np.sqrt(r)
                            if r < 1e6:
                                r = 1
                            potential_map[k][m] += (1 / (outer_r - inner_r)) * \
                                                   ((inner_r * outer_r * (alpha - 1)) / r + outer_r - inner_r * alpha)
    return potential_map


def random_maze_gen():
    map_size = np.random.randint(20, 101)
    # map_size = 100
    new_map = np.zeros((map_size, map_size), dtype=np.int64)
    for i in range(map_size):
        for j in range(map_size):
            new_map[i, j] = np.random.randint(0, 500) / 495
    # 외부 벽 생성
    # for i in range(map_size):
        # new_map[i, 0] = 1
        # new_map[0, i] = 1
        # new_map[i, map_size - 1] = 1
        # new_map[map_size - 1, i] = 1
    new_map[0:2, 0:2] = 0
    new_map[map_size - 1:map_size - 3, map_size - 1:map_size - 3] = 0
    return new_map.tolist()