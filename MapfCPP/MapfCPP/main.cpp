#include <iostream>

#include "AstarAgent.h"
#include "AStar.h"
#include "AStarMapGen.h"

#include "Agent.h"
#include "CTNode.h"
#include "Constraints.h"
#include "Assigner.h"
#include "Hungarian.h"
#include "planner.h"


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


int main()
{
	vector<p> start = { {1, 2}, {2, 1} };
	vector<p> goal = { {18, 17}, {19, 18} };
	vector<p> static_obstacle = { {10, 9} };
	Planner planner(1, 1, static_obstacle);
	vec2PInt result = planner.plan(start, goal, 200, 100, false);
	return 0;
}