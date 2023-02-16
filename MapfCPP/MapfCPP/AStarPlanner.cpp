#include "AStarPlanner.h"

//단일 알고리즘 반환
Path AStarPlanner(p start, p goal, std::map<int, std::set<p>> conf_path, std::map<int, std::set<std::pair<int, int>>> semi_dynamic_obstacles, int max_iter, bool debug) {
    Map map = test_maze_gen();
    Map potential_map = potential_map_generator(map);
    AStarAgent A(start, start, goal, "A");
    A.set_astar_path(map, potential_map, "init");
    bool flag = true;
    while (flag)
    {
        A.set_astar_path(map, potential_map, "renew");
        flag = A.move_path();
        A.set_position();
        A.aTimePlus();
    }
    return A.position;
}