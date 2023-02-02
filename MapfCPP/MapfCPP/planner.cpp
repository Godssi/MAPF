#include <vector>
#include <tuple>
#include <unordered_map>
#include <functional>
#include <thread>
#include <queue>
#include <iostream>
#include <utility>
#include <mutex>
//#include "planner.h"

class Planner
{
public:
	int robot_radius;
	/*STPlanner st_planner;*/
	Planner(int grid_size, int robot_radius, std::vector<std::pair<int, int>> static__obstacle) : robot_radius(robot_radius) /*st_planner(static__obstacle)*/ {}

};


