"""
11/17 공부 내용
1. code 분석이 완벽하게 필요하다 아직 잘 모르는 부분이 필요(민철, 지호) --> 11/17일 확인
2. 대각선 요소에 g 값 최신화에 거리를 1로 설정하였다. 이것을 대각선으로 이동할 때 if 문을 이용하여 abs(x) + abs(y) = 2 일 때는 거리를 root(2)가 되도록 다시 설정
--> 수정 완료 (11/17일)
3. 구상한 heuristic 함수를 적용하는 것을 해보자.
--> 구성한 heuristic 함수 적용, 아직 안전성을 고려한 가중치 값 수정
4. 결과를 시각화 : 경로를 map 상에 표시해서 결과 자체를 map 으로 보여주기 ( 이동 경로 표시 : *) --> (11/16일 수정 완료)
5. 여러 가지 path를 만드는 대신 편향성이 있는 path를 선정할 수 있는 code 작성 필요
--> 랜덤 경유지 설정 후 경로 탐색 or openList에서 노드 하나 뺸 다음 해당 노드에서 새로운 경로 설정 but 최단 경로로 x 편향성있는 다른 경로 설정
6. openList 갱신 --> 공부 필요
7. 직선으로 움직일 때 꼬불꼬불한 길 보다는 완전 직선에 가까운 경로를 만들 수 있도록 경로 설정 편향성 만들기
"""

"""
* 구성한 heuristic 함수 
1. 움직이는 장애물을 map 상에서 2로 구분을 할 것이다.
2. 움직이는 장애물 = MOB 통일 
3. 구성에서 중요한 점은 heuristic 함수에서의 값을 구할 떄 Robot 과 goal point 직선 과 MOB 의 직선 거리를 따진다.
4. 3번에 따라 직선거리가 안전한 거리 내인지 아닌지에 따라 가중치를 바꿔준다.
"""


"""
* 코드 분석에서 이해가 어려운 부분 & 수정해야하는 부분--> 헷갈리는 내용 해당 line 윗 부분에 내용 적어놓음
1. 135line ~ 146line
"""


import numpy as np
from map_gen import potential_map_generator
from heuristic_func import heuristic, heuristic_e


class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent  # 이전 노드
        self.position = position  # 현재 위치

        self.f = 0
        self.g = 0
        self.h = 0

    def __eq__(self, other):
        return self.position == other.position


"""
*Node 설명
1. position 은 x-y 좌표계를 사용하여 파악
2. parent 는 이전 노드인데 이전 노드
--> 이전 노드를 저장해두는 이유?  
2. f : 출발 지점에서 목적지까지의 총 cost 합 , f = g + h 이다. 우리는 f가 가장 작은 경로를 찾아내는 것이 목표이다.
3. g : 현재 노드에서 출발 지점까지의 총 cost
4. h : Heuristic(휴리스틱)함수, 현재 노드에서 목적지까지의 추정 거리
"""


def aStar(maze, start, end, origin_maze):
    # startNode 와 endNode 초기화
    startNode = Node(None, start)
    endNode = Node(None, end)

    # openList, closedList 초기화
    openList = []
    closedList = []

    """
    OpenList : 최단 경로를 분석하기 위한 상태값들이 계속 경신되는 배열
    ClosedList : 처리가 완료된 노드를 담아 두기 위한 배열
    -> 우리는 결국 ClosedList 에 추가되는 노드들 중에서 endNode 에 해당하는 Node 를 끝에 
    얻은 후에 그것의 parentNode 를 역추적하는 방식으로 경로를 찾는 것이다. 
    """

    openList.append(startNode)

    # endNode 를 찾을 때까지 실행
    while openList:
        # 현재 노드 지정
        # currentNode : 탐색을 시작할 Node 로 그 주위와 연결되어있는 Node를 찾기위해서 선정하는 하나의 Node일 뿐이다.
        currentNode = openList[0]
        currentidx = 0

        # 이미 같은 노드가 openList 에 있고, g 값이 더 크면
        # currentNode 를 openList 안에 있는 값으로 교체
        # 밑의 code 의미 : currentNode 자체를 바꾸는 것이다.
        # 그러니까 최적화를 위해서 openList에 포함된 모든 Node에 대해서 주위 Node에 대한 경로찾기 실행 x, 가장 작은 f 값을 가지고 있는 Node에 대해서만 경로찾기 실행
        for index, item in enumerate(openList):
            if item.f < currentNode.f:
            # if item.g > currentNode.g:
                currentNode = item
                currentidx = index

        # openList 에서 제거하고 closedList에 추가
        openList.pop(currentidx)
        closedList.append(currentNode)

        # 현재 노드가 목적지면 current.position 추가하고
        # current 의 부모로 이동
        """
        시각화를 할 수 있는 부분으로 역추적을 통해 경로를 알아내기 때문에
        역추적 과정에서 maze 자체에서 currentNode에 해당하는 좌표에 있는 maze에 *로 경로를 표시
        """
        if currentNode == endNode:
            current = currentNode
            while current is not None:
                x, y = current.position
                origin_maze[x][y] = '*'
                current = current.parent
            return origin_maze

        children = []
        # 인접한 x-y 좌표 전부
        # 인접한 x-y 좌표를 전부 확인하는 이유는? grid 형식의 a* algorithm
        # 인접해있는 모든 노드로 움직일 수 있기 때문에 childNode가 모든 방향의 Node가 될 수 있다.
        # newPosition : currentNode 와 인접해있는 Node 의 x,y를 파악하는 것
        for newPosition in [(0, -1), (0, 1), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
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
            if maze[nodePostion[0]][nodePostion[1]] >= 100000:
                continue

            new_node = Node(currentNode, nodePostion)  # 부모가 currentNode, position이 nodePosition임
            children.append(new_node)  # 미로인덱스 외, 음수, 장애물 등의 이상 값이 아닌경우 children에 new_node로 저장

        # 자식들 모두 loop
        for child in children:
            # 자식이 closedList 에 있으면 continue
            # 자식이 closedList 에 없는 경우에만 openList에 넣고 F score를 비교해서 closedList에 넣는다.
            if child in closedList:
                continue

            # f, g, h 값 업데이트
            # 이 부분에서 g 값을 설정할 때 대각선으로 움직일 때는 대각선의 길이를 더 해줘야한다. 따라서 if 문을 통한 구성 필요.
            # 여기서 대각선 길이를 2 ** (1/2)로 넣는 것 보다는 그냥 2로 넣는 것이 더 좋을 것 같다. --> 데이터의 크기 및 소수점 비교에서 오류가 생길 수 있다.
            if (child.position[0] != currentNode.position[0]) and (child.position[1] != currentNode.position[1]):
                child.g = currentNode.g + 2
            else:
                child.g = currentNode.g + 1

            child.h = heuristic(child, endNode, maze)
            # child.h = heuristic_e(child, endNode)

            child.f = child.g + child.h
            # print(child.h, child.g, child.f)

            """
            자식이 openList 에 있고, g값이 더 크면 continue
            --> 이 부분도 이미 openList에 있는 자식노드의 g 값이 더 크다면 children에 있는 childNode로 바꿔줘야하는 것이 아닌가?
            이 부분을 아예 바꿔줘야할 것 같다. 
            for 문을 통해서 구문 자체를 새롭게 만드는 것이 필요
            """

            # openList : 아직 조사하지 않은 노드이다.
            # 우리는 openList 에 있는 Node들의 f 값을 비교해서 더 작은 Node들을 closedNode에 넣어야 한다.
            # 따라서 우리는 openList를 새롭게 갱신해줄 필요가 있다.
            # 갱신해주는 조건 : 같은 위치에 있는 Node를 비교하는 것. But openList에 들어가는 child는 g값이 작은 Node가 들어가야한다.
            # print(openList)
            for openNode in openList:
                # print(openNode)
                if (child.position == openNode.position) and (child.g < openNode.g) and not openList:
                    openList.pop(openNode)  # 여기서 해당 openNode를 openList에서 빼기

            openList.append(child)


def main():
    origin_maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 1, 0, 0, 1, 1, 0],
                   [0, 0, 0, 0, 1, 0, 0, 1, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 1, 1, 0]]
    potential_map = potential_map_generator(origin_maze)
    start = (0, 0)
    end = (9, 9)

    path = aStar(potential_map, start, end, origin_maze)  # path 자체가 maze에서 a* algorithm을 통해서 구한 경로

    # 밑의 코드는 path 자체를 시각화하여서 표현한 것이다!. 이것을 위에 시각화 부분을 수정.
    for i in range(10):
        for j in range(10):
            print(path[i][j], end=" ")
        print()


if __name__ == '__main__':
    main()
