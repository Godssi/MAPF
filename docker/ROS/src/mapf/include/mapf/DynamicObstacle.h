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

#include <mapf/AStar/Astar.h>

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
	vecPInt path;
    string dynamicObstacle_Name;
	int direct_vector;
	int present_idx;
	int after_idx;
	
	DynamicObstacle(string dynamicObstacle_Name, vector< pairInt> path, pairInt present_pos, pairInt after_pos) : dynamicObstacle_Name(dynamicObstacle_Name), path(path) 
	{
		present_idx = 0;
		after_idx = 1;
	};

	int hash() const;
	string str();
	string repr();
	bool operator==(const DynamicObstacle& other) const;
	bool operator<(const DynamicObstacle& other) const;
	friend std::ostream& operator<< (std::ostream& os, const DynamicObstacle& a);

	void Direction(vector< pairInt> path);
};

#endif