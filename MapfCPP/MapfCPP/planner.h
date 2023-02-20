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

#include "AStarPlanner.h"

#include "Agent.h"
#include "CTNode.h"
#include "Constraints.h"
#include "Assigner.h"

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


class Planner
{
public:
	int robot_radius;
	int low_level_max_iter;
	bool debug;

	int core = 0;
	int max_core = 2;

	AStarPlanner aStarPlanner;
	vecAgent agents;
	vector<pair<Agent, Agent>> combi;
	Planner(int grid_size, int robot_radius, vecPInt static_obstacle, int low_level_max_iter = 100,bool debug = false) : robot_radius(robot_radius), debug(debug), low_level_max_iter(low_level_max_iter) /*st_planner(static__obstacle)*/ {}
public:
	vec2PInt plan(vecPInt starts, vecPInt goals, int max_iter = 200, int low_level_max_iter = 100, bool debug = false);
	void search_node(CTNode& best, pair<vector<pairCTNode>, vector<vec2PInt>>& results);
	vecPAgent combination(vecAgent total_agent);
	pair<vecAgent, int> validate_paths(vecAgent agents, CTNode node);
	int safe_distance(map<Agent, vecPInt> solution, Agent agent_i, Agent agent_j);
	double dist(pairInt point1, pairInt point2);
	Constraints calculate_constraints(CTNode& node, Agent& constrainted_agent, Agent& unchanged_agent, int& time_of_conflict);
	map<int, setPInt> calculate_goal_times(CTNode& node, Agent& agent, vecAgent& agents);
	vecPInt calculate_path(Agent agent, Constraints constraints, map<int, setPInt> goal_times);
	vec2PInt reformat(vecAgent agents, map<Agent, vecPInt>& solution);
	void pad(map<Agent, vecPInt>& solution);

	void set_max_core();
};


#endif

