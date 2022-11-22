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
                         np.arange(node.position[0], goal.position[0] + 1, 1))).tolist()
    else:
        idx = []  # 추가 예정
    return idx


def heuristic(node, goal, MOB):  # Our heuristic function
    # !!!!일단은 MOB가 1개일 때만 생각 -> 여러개인 경우 for문을 이용해 w를 반복적으로 계산
    # 현 node와 goal 사이의 직선과 MOB의 직선거리 - 신발끈 공식 사용
    area = abs(
        node.position[0] * goal.position[1] + goal.position[0] * MOB.position[1] + MOB.position[0] * node.position[1]
        - goal.position[0] * node.position[1] - MOB.position[0] * goal.position[1] - node.position[0] * MOB.position[1])
    d = ((node.position[0] - goal.position[0]) ** 2 + (
                node.position[1] - goal.position[1]) ** 2) ** 0.5  # agent, goal 사이 거리 재사용!
    r = area / d
    # 가중치 계산
    p, alp, bet, k = 1, 1, 1, 1  # !!!값 수정!
    if r < alp:
        w = p
    elif r < bet:
        w = k * bet / r
    else:
        w = 1
    # 휴리스틱 값
    h = w * d
    return h
