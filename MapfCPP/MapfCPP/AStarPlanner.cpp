#include "AStarPlanner.h"

AStarPlanner::AStarPlanner()
{
    origin_map = MAP_GEN::test_maze_gen();
    static_potential_map = MAP_GEN::potential_map_generator(origin_map);
    origin = false;
}

void AStarPlanner::modify_potential_map(vector<DynamicObstacle>& dynamic_obstacle)
{
    dynamic_potential_map = MAP_GEN::dynamic_potential_map(origin_map, dynamic_obstacle);
}

void AStarPlanner::set_static_obstacle(const vecPInt& static_obstacle)
{
    for (auto& it : static_obstacle)
    {
        if (origin_map[it.first][it.second] == Ground)
            origin_map[it.first][it.second] = Static_Ob;
    }
}

void AStarPlanner::set_dynamic_obstacle(const vecPInt& dynamic_obstacle)
{
    for (auto& it : dynamic_obstacle)
    {
        for (auto& it : dynamic_obstacle)
        {
            if (origin_map[it.first][it.second] == Ground)
                origin_map[it.first][it.second] = Dynamic_Ob;
        }
    }
}

void AStarPlanner::set_low_level_max_iter(int low_level_max_iter)
{
    this->low_level_max_iter = low_level_max_iter;
}

Path AStarPlanner::aStarPlan(pairInt start, pairInt goal, const map<int, set<pairInt>>& conf_path, const map<int, set<pairInt>>& semi_dynamic_obstacles, vector<DynamicObstacle> dynamic_obstacle, int time_step)
{
    if (!origin)
        return AStar(start, goal, origin_map, static_potential_map, dynamic_potential_map, conf_path, semi_dynamic_obstacles, dynamic_obstacle, time_step, low_level_max_iter);
    else
        return AStar_Origin(start, goal, origin_map, static_potential_map, dynamic_potential_map, conf_path, semi_dynamic_obstacles, dynamic_obstacle, time_step, low_level_max_iter);
}