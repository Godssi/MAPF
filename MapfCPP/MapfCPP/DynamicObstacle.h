#ifndef DYANAMICOBSTACLE_H
#define DYANAMICOBSTACLE_H

#include <iostream>
#include <utility>
#include <string>
#include <vector>
#include <map>
#include <list>
#include <cmath>
#include <algorithm>
#include <functional>
#include "AStar.h"

// 방향을 하나의 int 값으로 설정하여서 조금 보기 편하게 설정
enum Direction {
	None,
	North,
	North_East,
	East,
	South_East,
	South,
	South_West,
	West,
	North_West
};

using namespace std;

class DynamicObstacle
{
public:
	vecPInt path; // 전체 경로를 저장하는 멤버 변수
	int direct_vector; // 속도 방향 벡터 변수
	string dynamicObstacle_Name;
	// 현재 위치와 그 다음 위치를 멤버 변수로 저장
	int present_idx;
	int after_idx;
	
	DynamicObstacle(string dynamicObstacle_Name, vector< pairInt> path, pairInt present_pos, pairInt after_pos) : dynamicObstacle_Name(dynamicObstacle_Name), path(path) 
	{
		present_idx = 0;
		after_idx = 1;
	};

	int hash() const;  // 각각의 DynamicObstacle을 구별하기 위한 함수와 함수값을 만드는 함수
	string str();
	string repr();
	bool operator == (const DynamicObstacle& other) const;
	bool operator< (const DynamicObstacle& other) const;
	friend std::ostream& operator<< (std::ostream& os, const DynamicObstacle& a);

	void Direction(vector< pairInt> path);
};

#endif