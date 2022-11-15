class Node:
    def __init__(self, parent=None,position=None):
        self.parent=parent
        self.position=position
        self.g=0
        self.h=0
        self.f=0
        
    def __eq__(self, other):
        return self.position == other.position

def heuristic(node,goal,D=1):
    dx=abs(node.position[0]-goal.position[0])
    dy=abs(node.position[1]-goal.position[1])
    return D*(dx**2+dy**2)
"""
def heuristic(node, goal, D=1, D2=2**0.5): #Diagonal Distance
    dx=abs(node.position[0]-goal.position[0])
    dy=abs(node.position[1]-goal.position[1])
    return D*(dx+dy)+(D2-2*D)*min(dx,dy)
"""
"""
def heuristic(node, goal, D=1):
    dx=abs(node.position[0]-goal.position[0])
    dy=abs(node.position[1]-goal.position[1])
    return D*(dx+dy)
"""
def aStar(maze, start, end):
    #시작, 끝 노드 초기화
    
    startNode=Node(None,start) #start는 시작 노드의 position
    endNode=Node(None,end)
    #openlist, closedlist
    openList=[]
    closedList=[]
    #openlist에 시작 노드 추가
    openList.append(startNode)
    
    #endNode 도달시까지 실행
    while openList: #openlist가 없어질때까지
        #현재 노드 정의
        currentNode=openList[0]
        currentIdx=0 
        

        #이미 같은 노드가 openlist에 있고, f값이 더 크면
        #currentNode를 openlist안에 있는 값으로 교체(f가 가장 작은 것 뽑음)
        for index, item in enumerate(openList): #enumerate=각 요소 인덱스 포함한 튜플로 만들어줌
            if item.f<currentNode.f: 
                currentNode = item
                currentIdx = index
        
        #openlist에서 closedlist로 currentNode를 이동시킴
        openList.pop(currentIdx)
        closedList.append(currentNode)
        
        #현재 노드가 타겟이면 current.position 추가하고
        #current의 부모로 이동
        if currentNode ==endNode:
            path=[]
            current=currentNode
            while current is not None:
                path.append(current.position)
                current=current.parent
            return path[::-1] #-1은 시작 position부터 뜨도록 reverse해줌
        
        
        
        children=[] #인접 노드 확인
        for newPosition in [(0,-1),(0,1),(-1,0),(1,0),(-1,-1),(-1,1),(1,-1),(1,1)]:
            #nodePosition 업데이트 - 주변 노드의 position
            nodePosition = (
                currentNode.position[0]+newPosition[0], #x
                currentNode.position[1]+newPosition[1]) #y
            
            #미로 maze index 안에 있는지 확인
            within_range_criteria=[
                nodePosition[0]>(len(maze)-1),
                nodePosition[0]<0,
                nodePosition[1]>(len(maze[len(maze)-1])-1),
                nodePosition[1]<0,
            ]
        
            if any(within_range_criteria):
                continue
            if maze[nodePosition[0]][nodePosition[1]]!=0:
                continue #이러면 다른 인접 노드로 가게됨
            new_node=Node(currentNode,nodePosition) #부모가 currentNode, position이 nodePosition임
            children.append(new_node) #미로인덱스 외, 음수, 장애물 등의 이상 값이 아닌경우 children에 new_node로 저장
        
        #자식들 모두 loop
        for child in children:
            #자식이 closedlist에 있으면 continue (에외처리)
            if child in closedList:
                continue
            # f,g,h값 업데이트
            child.g=currentNode.g+1 #부모로부터 1만큼 더간거니깐 1 더해줌
            child.h = heuristic(child,endNode)
            
            child.f=child.g+child.h
            
            #자식이 openlist에 있고, g값이 더 크다면(f값이 아니라?) continue
            if len([openNode for openNode in openList if child == openNode and child.g>openNode.g])>0:
                continue #오픈리스트에 있는 노드보다 커서 리스트에 값이들어가면 요소가 존재해 0보다 커짐 ->그경우 continue로 예외처리
            
            openList.append(child)
                
def main():
    # 1은 장애물
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