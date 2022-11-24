import numpy as np
from H import heuristic, heuristic_d, heuristic_e, heuristic_m
from V import visualize_m, visualize_p


class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent  # 이전 노드
        self.position = position  # 현재 위치

        self.f = 0
        self.g = 0
        self.h = 0

    def __eq__(self, other):
        return self.position == other.position

##############################################################################


def aStar(maze, start, end, MOB_list):
    # startNode 와 endNode 초기화
    startNode = Node(None, start)
    endNode = Node(None, end)

    # openList, closedList 초기화
    openList = []
    closedList = []

    openList.append(startNode)

    while openList:

        currentNode = openList[0]
        currentidx = 0

        for index, item in enumerate(openList):
            if item.f < currentNode.f:
                # if item.g > currentNode.g:
                currentNode = item
                currentidx = index

        # openList 에서 제거하고 closedList에 추가
        openList.pop(currentidx)
        closedList.append(currentNode)

        if currentNode == endNode:
            path = []
            current = currentNode
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path

        children = []
        # 인접한 x-y 좌표 전부
        # 인접한 x-y 좌표를 전부 확인하는 이유는? grid 형식의 a* algorithm
        # 인접해있는 모든 노드로 움직일 수 있기 때문에 childNode가 모든 방향의 Node가 될 수 있다.
        # newPosition : currentNode 와 인접해있는 Node 의 x,y를 파악하는 것
        for newPosition in [(0, -1), (0, 1), (1, 0), (-1, 0)]:
            # 노드 위치 업데이트
            nodePostion = (
                currentNode.position[0] + newPosition[0],
                currentNode.position[1] + newPosition[1]
            )

            # 미로 maze index 범위 안에 있어야함
            within_range_criteria = [
                nodePostion[0] > (len(maze) - 1),
                nodePostion[0] < 0,
                nodePostion[1] > (len(maze[len(maze) - 1]) - 1),
                nodePostion[1] < 0
            ]

            # 하나라도 true라면 범위 밖임
            if any(within_range_criteria):
                continue

            # 장애물이 있으면 다른 위치 불러오기
            if maze[nodePostion[0]][nodePostion[1]] != 0:
                continue

            # 부모가 currentNode, position이 nodePosition임
            new_node = Node(currentNode, nodePostion)
            # 미로인덱스 외, 음수, 장애물 등의 이상 값이 아닌경우 children에 new_node로 저장
            children.append(new_node)

        # 자식들 모두 loop
        for child in children:
            # 자식이 closedList 에 있으면 continue
            # 자식이 closedList 에 없는 경우에만 openList에 넣고 F score를 비교해서 closedList에 넣는다.
            if child in closedList:
                continue

            # f, g, h 값 업데이트
            # 이 부분에서 g 값을 설정할 때 대각선으로 움직일 때는 대각선의 길이를 더 해줘야한다. 따라서 if 문을 통한 구성 필요.
            # 여기서 대각선 길이를 2 ** (1/2)로 넣는 것 보다는 그냥 2로 넣는 것이 더 좋을 것 같다. --> 데이터의 크기 및 소수점 비교에서 오류가 생길 수 있다.
            child.g = currentNode.g + 1

            child.h = heuristic(child, endNode, MOB_list)

            child.f = child.g + child.h

            for openNode in openList:
                if (child.position == openNode.position) and (child.g < openNode.g):
                    openList.remove(openNode)  # 여기서 해당 openNode를 openList에서 빼기

            openList.append(child)


def main():
    MOB_list = [Node(None, (2, 1)), Node(None, (5, 5))]
    maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 1, 1, 0]]
    start = (0, 0)
    end = (8, 8)
    visualize_m(maze, MOB_list)

    path = aStar(maze, start, end, MOB_list)
    visualize_p(maze, path)


if __name__ == '__main__':
    main()
