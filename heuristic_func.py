import numpy as np

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


def heuristic_e(node, goal, D=1, D2=2 ** 0.5):  # Euclidean Distance
    dx = abs(node.position[0] - goal.position[0])
    dy = abs(node.position[1] - goal.position[1])
    return D * (dx ** 2 + dy ** 2)


def get_index_to_goal(node, goal):
    idx = []
    if goal.position[0] - node.position[0] == goal.position[1] - node.position[1]:
        idx = np.vstack((np.arange(node.position[0], goal.position[0] + 1, 1),
                         np.arange(node.position[0], goal.position[0] + 1, 1))).T.tolist()
        return [idx]
    else:
        upper_idx = []
        lower_idx = []
        # 목적지가 1사분면에 있는 경우
        if node.position[0] <= goal.position[0] and node.position[1] <= goal.position[1]:
            for i in range(node.position[0], goal.position[0] + 1, 1):
                for j in range(node.position[1], goal.position[1] + 1, 1):
                    if (goal.position[1] - node.position[1] + 1) * (i - goal.position[0]) >= (goal.position[0] - node.position[0] + 1) * (j - goal.position[1]):
                        upper_idx.append([i, j])
                    if (goal.position[1] - node.position[1] + 1) * (i - goal.position[0]) <= (goal.position[0] - node.position[0] + 1) * (j - goal.position[1]):
                        lower_idx.append([i, j])
        # 목적지가 2사분면에 있는 경우
        elif node.position[0] >= goal.position[0] and node.position[1] <= goal.position[1]:
            for i in range(node.position[0], goal.position[0] - 1, -1):
                for j in range(node.position[1], goal.position[1] + 1, 1):
                    if (goal.position[1] - node.position[1] + 1) * (i - goal.position[0]) >= (goal.position[0] - node.position[0] - 1) * (j - goal.position[1]):
                        upper_idx.append([i, j])
                    if (goal.position[1] - node.position[1] + 1) * (i - goal.position[0]) <= (goal.position[0] - node.position[0] - 1) * (j - goal.position[1]):
                        lower_idx.append([i, j])
        # 목적지가 3사분면에 있는 경우
        elif node.position[0] >= goal.position[0] and node.position[1] >= goal.position[1]:
            for i in range(node.position[0], goal.position[0] - 1, -1):
                for j in range(node.position[1], goal.position[1] - 1, -1):
                    if (goal.position[1] - node.position[1] - 1) * (i - goal.position[0]) >= (goal.position[0] - node.position[0] - 1) * (j - goal.position[1]):
                        upper_idx.append([i, j])
                    if (goal.position[1] - node.position[1] - 1) * (i - goal.position[0]) <= (goal.position[0] - node.position[0] - 1) * (j - goal.position[1]):
                        lower_idx.append([i, j])
        # 목적지가 4사분면에 있는 경우
        elif node.position[0] <= goal.position[0] and node.position[1] >= goal.position[1]:
            for i in range(node.position[0], goal.position[0] + 1, 1):
                for j in range(node.position[1], goal.position[1] - 1, -1):
                    if (goal.position[1] - node.position[1] - 1) * (i - goal.position[0]) >= (goal.position[0] + node.position[0] + 1) * (j - goal.position[1]):
                        upper_idx.append([i, j])
                    if (goal.position[1] - node.position[1] - 1) * (i - goal.position[0]) <= (goal.position[0] + node.position[0] + 1) * (j - goal.position[1]):
                        lower_idx.append([i, j])
        return [upper_idx, lower_idx]


def heuristic(node, goal, maze):  # Our heuristic function
    # !!!!일단은 MOB가 1개일 때만 생각 -> 여러개인 경우 for문을 이용해 w를 반복적으로 계산
    # 현 node와 goal 사이의 직선과 MOB의 직선거리 - 신발끈 공식 사용
    h = heuristic_e(node, goal)
    idx = get_index_to_goal(node, goal)
    # 대각선이 존재하는 경우
    if len(idx) == 1:
        for i in range(len(idx[0])):
            h += maze[idx[0][i][0]][[idx[0][i][1]]][0]
            h = h / len(idx[0]) * np.sqrt(h)        # 경로상의 node 수 보정
    else:
        h1 = 0; h2 = 0
        for i in range(len(idx[0])):
            h1 += maze[idx[0][i][0]][[idx[0][i][1]]][0]
        for i in range(len(idx[1])):
            h2 += maze[idx[1][i][0]][[idx[1][i][1]]][0]
        if len(idx[0]) != 0 and len(idx[1]) != 0 :
            h1 = (h1 / len(idx[0])) * np.sqrt(h)        # 경로상의 node 수 보정
            h2 = (h2 / len(idx[1])) * np.sqrt(h)        # 경로상의 node 수 보정
        h += min(h1, h2)
    if type(h) is not int:
        h = int(h.astype(np.int64))
    if h <= 0:
        h = 0
    return h


# class Node:
#     def __init__(self, parent=None, position=None):
#         self.parent = parent  # 이전 노드
#         self.position = position  # 현재 위치
#
#         self.f = 0
#         self.g = 0
#         self.h = 0
#
#     def __eq__(self, other):
#         return self.position == other.position
#
#
# from map_gen import potential_map_generator
#
#
# if __name__ == '__main__':
#     np.set_printoptions(linewidth=np.inf)
#     maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 1, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 1, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 1, 1, 0]]
#     potential_map = potential_map_generator(maze)
#     print(heuristic(Node(position=(0, 0)), Node(position=(8, 9)), potential_map))
#     print(get_index_to_goal(Node(position=(1, 2)), Node(position=(-2, 4))))
