#include "AStarPlanner.h"

AStarPlanner::AStarPlanner()
{
	origin_map = MAP_GEN::test_maze_gen1();
	potential_map = MAP_GEN::potential_map_generator(origin_map);
}

void AStarPlanner::modify_potential_map()
{
	MAP_GEN::modify_potential_map(origin_map, potential_map);
}

void AStarPlanner::set_static_obstacle(const vecPInt& static_obstacle)
{
	for (auto& it : static_obstacle)
	{
		if (origin_map[it.first][it.second] == 0)
			origin_map[it.first][it.second] = 4;
	}
}

void AStarPlanner::set_low_level_max_iter(int low_level_max_iter)
{
	this->low_level_max_iter = low_level_max_iter;
}

Path AStarPlanner::aStarPlan(pairInt start, pairInt goal, const map<int, set<pairInt>>& conf_path, const map<int, set<pairInt>>& semi_dynamic_obstacles, bool debug)
{
	return AStar(start, goal, origin_map, potential_map, conf_path, semi_dynamic_obstacles, low_level_max_iter);
}