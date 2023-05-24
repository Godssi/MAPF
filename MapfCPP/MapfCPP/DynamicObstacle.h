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

using namespace std;

typedef pair<int, int> pairInt;
typedef vector<pairInt> vecPInt;
typedef vector<vecPInt> vec2PInt;

typedef vector<pairInt> Path;

//  direction??int 값으�??�정?�과 ?�시??계산
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

class DynamicObstacle
{
public:
	vecPInt DoB_path;
	vector<int> direct_vector;
	string dynamicObstacle_Name;
	int present_idx;
	int after_idx;

	DynamicObstacle(string dynamicObstacle_Name, vector<pairInt> path) : dynamicObstacle_Name(dynamicObstacle_Name), DoB_path(path)
	{
		present_idx = 0;
		after_idx = 1;
		Direction(DoB_path);
	};

	int hash() const;
	string str();
	string repr();
	bool operator==(const DynamicObstacle& other) const;
	bool operator<(const DynamicObstacle& other) const;
	friend std::ostream& operator<< (std::ostream& os, const DynamicObstacle& a);

	void Direction(vector<pairInt> path);
};

#endif