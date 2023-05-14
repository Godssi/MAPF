#ifndef DYNAMICOBSTACLE_H
#define DYNAMICOBSTACLE_H

#include <iostream>
#include <utility>
#include <string>
#include <vector>
#include <map>
#include <list>
#include <cmath>
#include <algorithm>
#include <functional>
#include <cstdlib>

using namespace std;

typedef pair<int, int> pairInt;
typedef vector<pairInt> vecPInt;
typedef vector<vecPInt> vec2PInt;
typedef vector<pairInt> Path;


class DynamicObstacle
{
public:
	double x_pos; // 동적 장애물의 x 위치
	double y_pos; // 동적 장애물의 y 위치
	double velocity; // 동적 장애물의 속도
	double velo_theta; // 동적 장애물의 이동 방향
	string dynamicObstacle_Name;
	
	DynamicObstacle(string dynamicObstacle_Name, double x, double y, double v) : dynamicObstacle_Name(dynamicObstacle_Name), x_pos(x), y_pos(y), velocity(v), velo_theta(0) {};

	int hash() const;
	string str();
	string repr();
	bool operator==(const DynamicObstacle& other) const;
	/*bool operator<(const dynamicobstacle& other) const;*/
	friend std::ostream& operator<< (std::ostream& os, const DynamicObstacle& a);

	// 동적 장애물이 움직일 방향을 정하는 함수 (랜덤 함수를 사용해서 정의)
	void Random_Theta();
	// 동적 장애물의 위치를 수정하는 함수
	void Move();
};

#endif