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

// ������ �ϳ��� int ������ �����Ͽ��� ���� ���� ���ϰ� ����
enum Direction {
	None,
	East,
	North_East,
	North,
	North_West,
	West,
	South_West,
	South,
	South_East,
};

using namespace std;

class DynamicObstacle
{
public:
	vecPInt path; // ��ü ��θ� �����ϴ� ��� ����
	vector<int> direct_vector; // �ӵ� ���� ���� ����
	string dynamicObstacle_Name;
	// ���� ��ġ�� �� ���� ��ġ�� ��� ������ ����
	int present_idx;
	int after_idx;
	
	DynamicObstacle(string dynamicObstacle_Name, vector<pairInt> path) : dynamicObstacle_Name(dynamicObstacle_Name), path(path) 
	{
		present_idx = 0;
		after_idx = 1;
	};

	int hash() const;  // ������ DynamicObstacle�� �����ϱ� ���� �Լ��� �Լ����� ����� �Լ�
	string str();
	string repr();
	bool operator==(const DynamicObstacle& other) const;
	bool operator<(const DynamicObstacle& other) const;
	friend std::ostream& operator<< (std::ostream& os, const DynamicObstacle& a);

	void Direction(vector< pairInt> path);
};

#endif