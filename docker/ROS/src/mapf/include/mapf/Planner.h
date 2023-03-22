#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <vector>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <map>
#include <functional>
#include <thread>
#include <utility>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <thread>

#include <mapf/Agent.h>
#include <mapf/CTNode.h>
#include <mapf/Constraints.h>
#include <mapf/Assigner.h>
#include <mapf/AStar/AStarPlanner.h>

using namespace std;

typedef long long ll;
typedef pair<int, int> pairInt;
typedef pair<CTNode, CTNode> pairCTNode;
typedef pair<Agent, Agent> pairAgent;
typedef vector<Agent> vecAgent;
typedef vector<CTNode> vecCTNode;
typedef vector<pairInt> vecPInt;
typedef vector<pairAgent> vecPAgent;
typedef vector<vecPInt> vec2PInt;
typedef set<pairInt> setPInt;
typedef vector<pair<vecPInt, int>> dynamicOb;


class Planner
{
private:
	int robot_radius;

	int core = 0;
	int max_core = 2;

	AStarPlanner aStarPlanner;
	vecPInt starts;
	vecPInt goals;
	vecAgent agents;

	Map static_potential_map;
	dynamicOb dynamic_obstacle;

	map<Agent, vecPInt> planResults;
	vector<pair<Agent, Agent>> combi;

public:
	Planner() {}
	Planner(int robot_radius, vecPInt static_obstacle, int low_level_max_iter = 1000) :
		robot_radius(robot_radius)
	{
		aStarPlanner.set_static_obstacle(static_obstacle);
	}
	Planner(vecPInt starts, vecPInt goals, int robot_radius, vecPInt static_obstacle, dynamicOb dynamic_obstacle, int low_level_max_iter = 1000) :
		starts(starts), goals(goals), robot_radius(robot_radius)
	{
		aStarPlanner.set_static_obstacle(static_obstacle);
		this->static_potential_map = aStarPlanner.get_static_potential_map();
		this->dynamic_obstacle = dynamic_obstacle;
	}

	// MAPF setting
	vec2PInt plan(int max_iter = 200, int low_level_max_iter = 1000);
	vec2PInt plan(vecPInt starts, vecPInt goals, int max_iter = 200, int low_level_max_iter = 1000);
	void search_node(CTNode& best, pair<vector<pairCTNode>, vector<vec2PInt>>& results, mutex& mtx);
	vecPAgent combination(vecAgent total_agent);
	pair<vecAgent, int> validate_paths(vecAgent agents, CTNode node);
	int safe_distance(map<Agent, vecPInt> solution, Agent agent_i, Agent agent_j);
	double dist(pairInt point1, pairInt point2);
	Constraints calculate_constraints(CTNode& node, Agent& constrainted_agent, Agent& unchanged_agent, int& time_of_conflict);
	map<int, setPInt> calculate_goal_times(CTNode& node, Agent& agent, vecAgent& agents);
	vecPInt calculate_path(Agent agent, Constraints& constraints, map<int, setPInt> goal_times);
	vecPInt recalculate_path(Agent agent, Constraints& constraints, map<int, setPInt> goal_times, int& time_of_conflict);
	vec2PInt reformat(vecAgent agents, map<Agent, vecPInt>& solution);
	void pad(map<Agent, vecPInt>& solution);

	// Planner setting
	void set_planner(vecPInt starts, vecPInt goals, int robot_radius, vecPInt static_obstacle, dynamicOb dynamic_obstacle, int low_level_max_iter = 1000);
	void set_planner(int robot_radius = 1, int low_level_max_iter = 1000);
	void set_starts(vecPInt starts);
	void set_goals(vecPInt goals);
	vecPInt get_starts();
	vecPInt get_goals();
	bool validate_agent_position();
	void modify_potential_map();
	Map get_static_potential_map() { return static_potential_map; }
	AStarPlanner get_aStarPlanner() { return aStarPlanner; }
	void set_max_core();
	void set_max_core(int n_core);
	int get_max_core() { return max_core; };
};


#endif