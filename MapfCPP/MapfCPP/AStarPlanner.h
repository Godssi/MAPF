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

Path AStarPlanner(p start, p goal, std::map<int, std::set<p>> conf_path, std::map<int, std::set<std::pair<int, int>>> semi_dynamic_obstacles, int max_iter, bool debug);

#endif