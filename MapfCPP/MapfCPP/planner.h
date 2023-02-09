#ifndef PLANNER_H
#define PLANNER_H

#include<unordered_set>
#include <vector>
#include <tuple>
#include <unordered_map>
#include <functional>
#include <thread>
#include <queue>
#include <iostream>
#include <utility>
#include <mutex>
#include <map>
#include "Agent.h"
#include "CTNode.h"
#include "Constraints.h"
#include "Assigner.h"

class Planner
{
public:
	int robot_radius;
	int low_level_max_iter;
	bool debug;
	std::vector<Agent> agents;
	/*STPlanner st_planner;*/
	Planner(int grid_size, int robot_radius, std::vector<std::pair<int, int>> static__obstacle) : robot_radius(robot_radius) /*st_planner(static__obstacle)*/ {}

public:
	std::map<Agent, std::vector<int, int>> plan(std::vector<std::pair<int, int>> starts, std::vector<std::pair<int, int>> goals, int max_iter = 200, int low_level_max_iter = 100, int max_process = 10, bool debug = false);
	void search_node(CTNode best, std::vector<std::pair<CTNode, CTNode>>& results);
	unordered_set<Agent, Agent, int> validate_paths(std::vector<Agent> agents, CTNode node);
	int safe_distance(std::map<Agent, std::vector<std::pair<int, int>>> solution, Agent agent_i, Agent agent_j);
	static double dist(std::pair<int, int>point1, std::pair<int, int> point2);
	Constraints calculate_constraints(CTNode& node, Agent& constrainted_agent, Agent& unchanged_agent, int& time_of_conflict);
	std::map<int, pair<int, int>> calculate_goal_times(CTNode& node, Agent& agent, std::vector<Agent>& agents); //이거는 calculate_path에 넣기위해 얘만 dict 형태
	std::vector<std::pair<int, int>> calculate_path(Agent agent, Constraints constraints, std::map<int, std::pair<int, int>> goal_times);///입력 잘 모르겠음
	static std::vector<std::vector<std::pair<int, int>>> reformat(std::vector<Agent> agents, std::map<Agent, std::vector<std::pair<int, int>>> solution);
	static map<Agent, std::vector<std::pair<int, int>>> pad(map<Agent, std::vector<std::pair<int,int>>> solution);
	std::vector<Agent> combination(std::vector<Agent> v, int n, int d, int cur);
};

#endif




