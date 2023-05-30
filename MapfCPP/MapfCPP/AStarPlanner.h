#ifndef ASTAR_APLANNER_H
#define ASTAR_APLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include <utility>
#include <map>
#include <set>
#include "DynamicObstacle.h"
#include "AStar.h"
#include "AStarMapGen.h"
#include "Astar_Origin.h"

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
	Map static_potential_map;
	Map dynamic_potential_map;
	int low_level_max_iter;
	bool origin;

public:
	AStarPlanner();
	void set_low_level_max_iter(int low_level_max_iter);

	void set_static_obstacle(const vecPInt& static_obstacle);
	void set_dynamic_obstacle(const vecPInt& dynamic_obstacle);
	void modify_potential_map(vector<DynamicObstacle>& dynamic_obstacle);
	Path aStarPlan(pairInt start, pairInt goal, const map<int, set<pairInt>>& conf_path, const map<int, set<pairInt>>& semi_dynamic_obstacles, vector<DynamicObstacle> dynamic_obstacle, int time_step);

	Map get_origin_map() { return origin_map; }
	Map get_static_potential_map() { return static_potential_map; }
	Map get_dynamic_potential_map() { return dynamic_potential_map; }
	void set_origin_map(Map& map) { origin_map = map; }
	void set_origin_path(bool origin) { this->origin = origin; }
};

#endif