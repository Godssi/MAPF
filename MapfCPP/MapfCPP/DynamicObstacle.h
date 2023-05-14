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
	double x_pos; // ���� ��ֹ��� x ��ġ
	double y_pos; // ���� ��ֹ��� y ��ġ
	double velocity; // ���� ��ֹ��� �ӵ�
	double velo_theta; // ���� ��ֹ��� �̵� ����
	string dynamicObstacle_Name;
	
	DynamicObstacle(string dynamicObstacle_Name, double x, double y, double v) : dynamicObstacle_Name(dynamicObstacle_Name), x_pos(x), y_pos(y), velocity(v), velo_theta(0) {};

	int hash() const;
	string str();
	string repr();
	bool operator==(const DynamicObstacle& other) const;
	/*bool operator<(const dynamicobstacle& other) const;*/
	friend std::ostream& operator<< (std::ostream& os, const DynamicObstacle& a);

	// ���� ��ֹ��� ������ ������ ���ϴ� �Լ� (���� �Լ��� ����ؼ� ����)
	void Random_Theta();
	// ���� ��ֹ��� ��ġ�� �����ϴ� �Լ�
	void Move();
};

#endif