#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <vector>
#include <string>
#include <utility>
#include <algorithm>

#include "AStarNode.h"
#include "Heuristic_func.h"

using namespace std;
typedef long long ll;
typedef pair<int, int> p;
typedef vector<p> Path;
typedef vector<vector<ll>> Map;

Path AStar(p start, p end, Map map, Map potential_map);

#endif