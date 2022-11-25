class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent  # 이전 노드
        self.position = position  # 현재 위치

        self.f = 0
        self.g = 0
        self.h = 0

    def __eq__(self, other):
        return self.position == other.position