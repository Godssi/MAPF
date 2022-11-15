"""
11/15 회의 내용
1. code 분석이 완벽하게 필요하다 아직 잘 모르는 부분이 필요(민철, 지호)
2. 대각선 요소에 g 값 최신화에 거리를 1로 설정하였다. 이것을 대각선으로 이동할 때 if 문을 이용하여 abs(x) + abs(y) = 2 일 때는 거리를 root(2)가 되도록 다시 설정
3. 구상한 heuristic 함수를 적용하는 것을 해보자.
4. 결과를 시각화 : 경로를 map 상에 표시해서 결과 자체를 map 으로 보여주기 ( 이동 경로 표시 : *)
"""

"""
* 구성한 heuristic 함수 
1. 움직이는 장애물을 map 상에서 2로 구분을 할 것이다.
2. 움직이는 장애물 = MOB 통일 
3. 구성에서 중요한 점은 heuristic 함수에서의 값을 구할 떄 Robot 과 goal point 직선 과 MOB 의 직선 거리를 따진다.
4. 3번에 따라 직선거리가 안전한 거리 내인지 아닌지에 따라 가중치를 바꿔준다.
"""

"""
* 점과 직선 사이의 거리를 구하는 코드

def dist(P, A, B):  P, A, B 자체를 node 로 생각하여 재구성 필요
    area = abs ( (A.x - P.x) * (B.y - P.y) - (A.y - P.y) * (B.x - P.x) )
    AB = ( (A.x - B.x) ** 2 + (A.y - B.y) ** 2 ) ** 0.5
    return ( area / AB )
    
"""

##############################################################################
#Node Class###################################################################
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
2. f : 출발 지점에서 목적지까지의 총 cost 합
3. g : 현재 노드에서 출발 지점까지의 총 cost
4. h : Heuristic(휴리스틱)함수, 현재 노드에서 목적지까지의 추정 거리
"""

##############################################################################
#Heuristic Function###########################################################
def heuristic_m(node, goal, D=1): #Manhattan Distance
    dx=abs(node.position[0]-goal.position[0])
    dy=abs(node.position[1]-goal.position[1])
    return D*(dx+dy)

def heuristic_d(node, goal, D=1, D2=2**0.5): #Diagonal Distance
    dx=abs(node.position[0]-goal.position[0])
    dy=abs(node.position[1]-goal.position[1])
    return D*(dx+dy)+(D2-2*D)*min(dx,dy)

def heuristic_e(node, goal, D=1, D2=2 ** 0.5):  #Euclidean Distance
    dx = abs(node.position[0] - goal.position[0])
    dy = abs(node.position[1] - goal.position[1])
    return D*(dx**2+dy**2)

def heuristic(node, goal, MOB): #Our heuristic function
    #!!!!일단은 MOB가 1개일 때만 생각
    # 현 node와 goal 사이의 직선과 MOB의 직선거리 - 신발끈 공식 사용
    area = abs(node.position[0]*goal.position[1]+goal.position[0]*MOB.position[1]+MOB.position[0]*node.position[1]
               -goal.position[0]*node.position[1]-MOB.position[0]*goal.position[1]-node.position[0]*MOB.position[1])
    d = ( (node.position[0] - goal.position[0]) ** 2 + (node.position[1] - goal.position[1]) ** 2 ) ** 0.5 # agent, goal 사이 거리 재사용!
    r=area / d
    # 가중치 계산
    p, alp, bet, k= 1,1,1,1 #!!!값 수정!
    if r<alp:
        w=p
    elif r<bet:
        w=k*bet/r
    else:
        w=1
    # 휴리스틱 값
    h=w*d
    return h
        

##############################################################################
#aStar function - path finding function#######################################
def aStar(maze, start, end):
    # startNode 와 endNode 초기화
    startNode = Node(None, start)
    endNode = Node(None, end)

    # openList, closedList 초기화
    openList = []
    closedList = []

    openList.append(startNode)

    # endNode 를 찾을 때까지 실행
    while openList:
        # 현재 노드 지정
        currentNode = openList[0]
        currentidx = 0

        # 이미 같은 노드가 openList 에 있고, f 값이 더 크면
        # currentNode 를 openList 안에 있는 값으로 교체
        for index, item in enumerate(openList):
            if item.f < currentNode.f:
                currentNode = item
                currentidx = index

        # openList 에서 제거하고 closedList에 추가
        openList.pop(currentidx)
        closedList.append(currentNode)

        # 현재 노드가 목적지면 current.position 추가하고
        # current 의 부모로 이동
        if currentNode == endNode:
            path = []
            current = currentNode
            while current is not None:
                x, y = current.position
                maze[x][y] = 7
                path.append(current.position)
                current = current.parent
            return path[::-1]

        children = []
        # 인접한 x-y 좌표 전부
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
            if maze[nodePostion[0]][nodePostion[1]] != 0:
                continue

            new_node = Node(currentNode, nodePostion)
            children.append(new_node)

        # 자식들 모두 loop
        for child in children:
            # 자식이 closedList 에 있으면 continue
            if child in closedList:
                continue

            # f, g, h 값 업데이트
            #!!!!!!!!!!!!!!!!!!!!!!!!!!!! g값 최신화에서 대각선 방향은 루트2로
            child.g = currentNode.g + 1 
            #!!!!!!!!!!!!!!!!!!!!!!!!!!!
            
            child.h = heuristic_e(child,endNode)
            child.f = child.g + child.h
            # print(child.h, child.g, child.f)

            # 자식이 openList 에 있고, g값이 더 크면 continue
            if len([openNode for openNode in openList
                    if child == openNode and child.g > openNode.g]) > 0:
                continue

            openList.append(child)


def main():
    maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 1, 1, 0],
            [0, 0, 0, 0, 1, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 1, 0]]

    start = (0, 0)
    end = (9, 9)

    path = aStar(maze, start, end)
    print(path)


if __name__ == '__main__':
    main()