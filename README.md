Multi Agent Path Finding (MAPF)
===============================

> ## 11/17 공부 내용
> 1. code 분석이 완벽하게 필요하다 아직 잘 모르는 부분이 필요(민철, 지호)  
> &rarr; 11/17일 확인
> 2. 대각선 요소에 g 값 최신화에 거리를 1로 설정하였다. 이것을 대각선으로 이동할 때, if 문을 이용하여 abs(x) + abs(y) = 2 일 때는 거리를 root(2)가 되도록 다시 설정  
> &rarr; 수정 완료 (11/17일)
> 3. 구상한 heuristic 함수를 적용하는 것을 해보자.  
> &rarr; 구성한 heuristic 함수 적용, 아직 안전성을 고려한 가중치 값 수정
> 4. 결과를 시각화 : 경로를 map 상에 표시해서 결과 자체를 map 으로 보여주기 ( 이동 경로 표시 : *)  
> &rarr; (11/16일 수정 완료)
> 5. 여러 가지 path를 만드는 대신 편향성이 있는 path를 선정할 수 있는 code 작성 필요  
> &rarr; 랜덤 경유지 설정 후 경로 탐색 or openList에서 노드 하나 뺸 다음 해당 노드에서 새로운 경로 설정 but 최단 경로로 x 편향성있는 다른 경로 설정
> 6. openList 갱신  
> &rarr; 공부 필요
> 7. 직선으로 움직일 때 꼬불꼬불한 길 보다는 완전 직선에 가까운 경로를 만들 수 있도록 경로 설정 편향성 만들기  