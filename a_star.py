from node import Node
from heuristic_func import heuristic, heuristic_e


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

    """
    OpenList : 최단 경로를 분석하기 위한 상태값들이 계속 경신되는 배열
    ClosedList : 처리가 완료된 노드를 담아 두기 위한 배열
    """

    # openList, closedList 초기화
    openList = []
    closedList = []

    openList.append(startNode)

    currentNode = openList[0]
    currentidx = 0

    # endNode 를 찾을 때까지 실행
    while openList:
        # 현재 노드 지정
        # currentNode : 탐색을 시작할 Node 로 그 주위와 연결되어있는 Node를 찾기위해서 선정하는 하나의 Node일 뿐이다.

        # 이미 같은 노드가 openList 에 있고, g 값이 더 크면
        # currentNode 를 openList 안에 있는 값으로 교체
        # 밑의 code 의미 : currentNode 자체를 바꾸는 것이다.
        # 그러니까 최적화를 위해서 openList에 포함된 모든 Node에 대해서 주위 Node에 대한 경로찾기 실행 x, 가장 작은 f 값을 가지고 있는 Node에 대해서만 경로찾기 실행

        openList = sorted(openList, key=lambda s: s.f)
        # print("openList: ", end=' ')
        # for i in range(len(openList)):
        #     if i < 15:
        #         print(openList[i].f, end=' ')
        # print()
        currentNode = openList[0]
        # currentidx = 0            # idx는 정렬을 통해 고정

        # openList 에서 제거하고 closedList에 추가
        openList.pop(currentidx)
        closedList.append(currentNode)

        # 현재 노드가 목적지면 current.position 추가하고
        # current 의 부모로 이동

        if currentNode == endNode:
            current = currentNode
            path_index = []
            while current is not None:
                x, y = current.position
                origin_maze[x][y] = '*'      # 역추적 과정에서 maze 자체에서 currentNode에 해당하는 좌표에 있는 maze에 *로 경로를 표시
                current = current.parent
                path_index.append([x, y])
            return origin_maze, path_index

        # newPosition : currentNode 와 인접해 있는 Node의 x,y

        children = []
        for newPosition in [(1, 0), (0, 1), (-1, 0), (0, -1)]:
            # 노드 위치 업데이트
            nodePostion = (
                currentNode.position[0] + newPosition[0],
                currentNode.position[1] + newPosition[1]
            )

            # 미로 maze index 범위 안에 있어야 함
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
            if origin_maze[nodePostion[0]][nodePostion[1]] == 1 or origin_maze[nodePostion[0]][nodePostion[1]] == 3:
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
            # print("openList len:", len(openList))
            for openNode in openList:
                if (child.position == openNode.position) and (child.g < openNode.g) and not openList:
                    openList.pop(openNode)  # 여기서 해당 openNode를 openList에서 빼기

            openList.append(child)
