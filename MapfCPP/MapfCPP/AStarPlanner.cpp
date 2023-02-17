#include "AStarPlanner.h"

//단일 알고리즘 반환
Path AStarPlanner(p start, p goal, map<int, set<p>> conf_path, map<int, set<p>> semi_dynamic_obstacles, int max_iter, bool debug) {

    Map map = test_maze_gen();
    Map potential_map = potential_map_generator(map);
    AStarAgent A(start, start, goal, "A");

    // cout << "\nconf path\n\n";
    // print_obstacles(conf_path);
    // cout << "\nsemi dynamic obstacles path\n\n";
    // print_obstacles(semi_dynamic_obstacles);

    /*
    A.set_astar_path(map, potential_map, conf_path, semi_dynamic_obstacles, "init");
    bool flag = true;
    while (flag)
    {
        A.set_astar_path(map, potential_map, conf_path, semi_dynamic_obstacles,"renew");
        flag = A.move_path();
        A.set_position();
        A.aTimePlus();
    }
    print_position(A.position);
    return A.position;
    */
    return AStarAgent::get_astar_path(start, goal, map, potential_map, conf_path, semi_dynamic_obstacles);
}