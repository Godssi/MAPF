#include "AStarPlanner.h"

AStarPlanner::AStarPlanner()
{
	origin_map = MAP_GEN::test_maze_gen();
	potential_map = MAP_GEN::potential_map_generator(origin_map);
}

Path AStarPlanner::aStarPlan(p start, p goal, map<int, set<p>> conf_path, map<int, set<p>> semi_dynamic_obstacles, int max_iter, bool debug)
{   
    return AStarAgent::get_astar_path(start, goal, origin_map, potential_map, conf_path, semi_dynamic_obstacles);
}

void AStarPlanner::modify_potential_map()
{
	potential_map = MAP_GEN::modify_potential_map(origin_map, potential_map);
}