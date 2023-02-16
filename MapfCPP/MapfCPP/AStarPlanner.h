#ifndef ASTAR_APLANNER_H
#define ASTAR_APLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include <utility>
#include <map>
#include <set>

#include "AStarAgent.h"
#include "AStar.h"
#include "AStarMapGen.h"

using namespace std;
typedef long long ll;
typedef pair<int, int> p;
typedef vector<p> Path;
typedef vector<vector<ll>> Map;

Path AStarPlanner(p start, p goal, map<int, set<p>> conf_path, map<int, set<p>> semi_dynamic_obstacles, int max_iter, bool debug);

#endif