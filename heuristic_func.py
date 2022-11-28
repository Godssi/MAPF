import numpy as np
from multiprocessing import Pool

##############################################################################
# Heuristic Function###########################################################


def heuristic_m(node, goal, D=1):  # Manhattan Distance
    dx = abs(node.position[0] - goal.position[0])
    dy = abs(node.position[1] - goal.position[1])
    return D * (dx + dy)


def heuristic_d(node, goal, D=1, D2=2 ** 0.5):  # Diagonal Distance
    dx = abs(node.position[0] - goal.position[0])
    dy = abs(node.position[1] - goal.position[1])
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)


def heuristic_e(node, goal):  # Euclidean Distance
    dx = abs(node.position[0] - goal.position[0])
    dy = abs(node.position[1] - goal.position[1])
    return np.sqrt(dx ** 2 + dy ** 2)


def get_index_to_goal(node, goal):
    idx = []
    if goal.position[0] - node.position[0] == goal.position[1] - node.position[1]:
        idx = np.vstack((np.arange(node.position[0], goal.position[0] + 1, 1),
                         np.arange(node.position[0], goal.position[0] + 1, 1),
                         np.sqrt(2) * (np.arange(node.position[0], goal.position[0] + 1, 1)))).astype(np.int64).T.tolist()
        return [idx]
    else:
        upper_idx = []
        lower_idx = []
        # 목적지가 1사분면에 있는 경우
        if node.position[0] <= goal.position[0] and node.position[1] <= goal.position[1]:
            for i in range(node.position[0], goal.position[0] + 1, 1):
                for j in range(node.position[1], goal.position[1] + 1, 1):
                    if (goal.position[1] - node.position[1] + 1) * (i - goal.position[0]) >= \
                            (goal.position[0] - node.position[0] + 1) * (j - goal.position[1]):
                        r = np.sqrt((i - node.position[0]) ** 2 + (j - node.position[1]) ** 2)
                        upper_idx.append([i, j, r])
                    if (goal.position[1] - node.position[1] + 1) * (i - goal.position[0]) <= \
                            (goal.position[0] - node.position[0] + 1) * (j - goal.position[1]):
                        r = np.sqrt((i - node.position[0]) ** 2 + (j - node.position[1]) ** 2)
                        lower_idx.append([i, j, r])
        # 목적지가 2사분면에 있는 경우
        elif node.position[0] >= goal.position[0] and node.position[1] <= goal.position[1]:
            for i in range(node.position[0], goal.position[0] - 1, -1):
                for j in range(node.position[1], goal.position[1] + 1, 1):
                    if (goal.position[1] - node.position[1] + 1) * (i - goal.position[0]) >= \
                            (goal.position[0] - node.position[0] - 1) * (j - goal.position[1]):
                        r = np.sqrt((i - node.position[0]) ** 2 + (j - node.position[1]) ** 2)
                        upper_idx.append([i, j, r])
                    if (goal.position[1] - node.position[1] + 1) * (i - goal.position[0]) <= \
                            (goal.position[0] - node.position[0] - 1) * (j - goal.position[1]):
                        r = np.sqrt((i - node.position[0]) ** 2 + (j - node.position[1]) ** 2)
                        lower_idx.append([i, j, r])
        # 목적지가 3사분면에 있는 경우
        elif node.position[0] >= goal.position[0] and node.position[1] >= goal.position[1]:
            for i in range(node.position[0], goal.position[0] - 1, -1):
                for j in range(node.position[1], goal.position[1] - 1, -1):
                    if (goal.position[1] - node.position[1] - 1) * (i - goal.position[0]) >= \
                            (goal.position[0] - node.position[0] - 1) * (j - goal.position[1]):
                        r = np.sqrt((i - node.position[0]) ** 2 + (j - node.position[1]) ** 2)
                        upper_idx.append([i, j, r])
                    if (goal.position[1] - node.position[1] - 1) * (i - goal.position[0]) <= \
                            (goal.position[0] - node.position[0] - 1) * (j - goal.position[1]):
                        r = np.sqrt((i - node.position[0]) ** 2 + (j - node.position[1]) ** 2)
                        lower_idx.append([i, j, r])
        # 목적지가 4사분면에 있는 경우
        elif node.position[0] <= goal.position[0] and node.position[1] >= goal.position[1]:
            for i in range(node.position[0], goal.position[0] + 1, 1):
                for j in range(node.position[1], goal.position[1] - 1, -1):
                    if (goal.position[1] - node.position[1] - 1) * (i - goal.position[0]) >= \
                            (goal.position[0] + node.position[0] + 1) * (j - goal.position[1]):
                        r = np.sqrt((i - node.position[0]) ** 2 + (j - node.position[1]) ** 2)
                        upper_idx.append([i, j, r])
                    if (goal.position[1] - node.position[1] - 1) * (i - goal.position[0]) <= \
                            (goal.position[0] + node.position[0] + 1) * (j - goal.position[1]):
                        r = np.sqrt((i - node.position[0]) ** 2 + (j - node.position[1]) ** 2)
                        lower_idx.append([i, j, r])
        return [upper_idx, lower_idx]


def heuristic(node, goal, maze):  # Our heuristic function
    h = heuristic_e(node, goal)
    idx = get_index_to_goal(node, goal)
    R = heuristic_e(node, goal)
    beta = 7  # 가까이 있는 장애물 보정계수
    # 대각선이 존재하는 경우
    if len(idx) == 1:
        h1 = 1
        for i in range(len(idx[0])):
            h1 += beta * (1 - ((min(idx[0][i][2], R) / (R + 1)) ** 3)) * maze[idx[0][i][0]][[idx[0][i][1]]][0]
        if len(idx[0]) == 0:
            return 1
        h *= h1 / len(idx[0]) * np.sqrt(R)        # 경로상의 node 수 보정
    else:
        h1 = 1
        h2 = 1
        for i in range(len(idx[0])):
            h1 += beta * (1 - ((min(idx[0][i][2], R) / (R + 1)) ** 3)) * maze[idx[0][i][0]][[idx[0][i][1]]][0]
        for i in range(len(idx[1])):
            h2 += beta * (1 - ((min(idx[1][i][2], R) / (R + 1)) ** 3)) * maze[idx[1][i][0]][[idx[1][i][1]]][0]
        if len(idx[0]) != 0 and len(idx[1]) != 0:
            h1 = (h1 / len(idx[0])) * np.sqrt(h)        # 경로상의 node 수 보정
            h2 = (h2 / len(idx[1])) * np.sqrt(h)        # 경로상의 node 수 보정
            # print("h1:", h1, "\nh2:", h2)
        h *= min(h1, h2)
    if type(h) is not int:
        h = int(h.astype(np.int64))
    if h <= 0:
        h = 1
    print("h:", h)
    return h
