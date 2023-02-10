#ifndef APLANNER_H
#define APLANNER_H

#include <iostream>
#include "aAgent.h"
#include "Astar.h"
#include "Map_gen.h"
#include <vector>
#include <string>
#include <utility>
#include "Astar.h"
#include <map>
#include <set>

using namespace std;
typedef long long ll;
typedef pair<int, int> p;
typedef vector<p> Path;
typedef vector<vector<ll>> Map;

void print_map(Map map);
Path aplanner(p start, p goal, std::map<int, std::set<p>> conf_path, std::map<int, std::set<std::pair<int, int>>> semi_dynamic_obstacles, int max_iter, bool debug);


#endif