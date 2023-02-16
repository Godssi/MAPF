#include "Agent.h"
#include "CTNode.h"
#include "Constraints.h"
#include "Assigner.h"
#include "Hungarian.h"
#include "planner.h"

#include <iostream>
#include "aAgent.h"
#include "Astar.h"
#include "Map_gen.h"


int main()
{
	vector<p> start = { {1, 2}, {2, 1} };
	vector<p> goal = { {18, 17}, {19, 18} };
	vector<p> static_obstacle = { {10, 9} };
	Planner planner(1, 1, static_obstacle);
	planner.plan(start, goal, 200, 100, false);
}