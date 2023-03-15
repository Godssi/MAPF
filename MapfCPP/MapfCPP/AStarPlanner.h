#ifndef ASTAR_APLANNER_H
#define ASTAR_APLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include <utility>
#include <map>
#include <set>

#include "AStar.h"
#include "AStarMapGen.h"

using namespace std;
typedef long long ll;
typedef pair<int, int> pairInt;
typedef vector<pairInt> vecPInt;
typedef vector<vecPInt> vec2PInt;
typedef vector<pairInt> Path;
typedef vector<vector<ll>> Map;


class AStarPlanner
{
private:
	Map origin_map;
	Map potential_map;
	int low_level_max_iter;

public:
	AStarPlanner();
	void set_low_level_max_iter(int low_level_max_iter);

	void set_static_obstacle(const vecPInt& static_obstacle);
	void set_dynamic_obstacle(const vecPInt& dynamic_obstacle);
	void modify_potential_map(const Map& static_potential_map);
	Path aStarPlan(pairInt start, pairInt goal, const map<int, set<pairInt>>& conf_path, const map<int, set<pairInt>>& semi_dynamic_obstacles, vector<pair<vecPInt, int>> dynamic_obstacle, int time_step);

	Map get_origin_map() { return origin_map; }
	Map get_potential_map() { return potential_map; }
	void set_origin_map(Map& map) { origin_map = map; }
};

#endif