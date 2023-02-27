#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <vector>
#include <string>
#include <utility>
#include <algorithm>
#include <map>
#include <set>

#include "AStarNode.h"
#include "Heuristic_func.h"

using namespace std;
typedef long long ll;
typedef pair<int, int> pairInt;
typedef vector<pairInt> vecPInt;
typedef vector<vecPInt> vec2PInt;

typedef vector<pairInt> Path;
typedef vector<vector<ll>> Map;

Path AStar(pairInt start, pairInt end, Map& origin_map, Map& potential_map, map<int, set<pairInt>> conf_path, map<int, set<pairInt>> semi_dynamic_obstacles);

#endif