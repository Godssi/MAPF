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


def heuristic(node, goal, MOB_list):  # Our heuristic function
    W = 1
    d = ((node.position[0] - goal.position[0]) ** 2 + (
        node.position[1] - goal.position[1]) ** 2) ** 0.5  # agent, goal 사이 거리 재사용!
    for MOB in MOB_list:
        area = abs(
            node.position[0] * goal.position[1] + goal.position[0] *
            MOB.position[1] + MOB.position[0] * node.position[1]
            - goal.position[0] * node.position[1] - MOB.position[0] * goal.position[1] - node.position[0] * MOB.position[1])
        if d == 0:
            break
        r = area / d
        # 가중치 계산
        p, alp, bet, k = 100000, 2**(0.5), 2, 1  # !!!값 수정!
        if r < alp:
            w = p
        elif r < bet:
            w = bet*k/r
        else:
            w = 1
            print(w,end=" ")
        W *= w
    # 휴리스틱 값
    print(W,node.position,)
    h = W * d
    return h

# 휴리스틱 문제가 있어어
