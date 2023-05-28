#ifndef ASTAR_ORIGIN_H
#define ASTAR_ORIGIN_H

#include <iostream>
#include <vector>
#include <string>
#include <utility>
#include <algorithm>
#include <map>
#include <set>

#include "DynamicObstacle.h"
#include "AStarNode.h"
#include "Heuristic_func.h"
#include "Astar.h"

using namespace std;
typedef long long ll;
typedef pair<int, int> pairInt;
typedef vector<pairInt> vecPInt;
typedef vector<vecPInt> vec2PInt;

typedef vector<pairInt> Path;
typedef vector<vector<ll>> Map;


Path AStar_Origin(pairInt start, pairInt end, Map& origin_map, Map& static_potential_map, Map& dynamic_potential_map, const map<int, set<pairInt>>& conf_path, const map<int, set<pairInt>>& semi_dynamic_obstacles, const vector<DynamicObstacle>& dynamic_obstacle, int time_step, int low_level_max_iter);

#endif
