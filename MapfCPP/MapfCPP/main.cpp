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

#include <ctime>

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

void print_vec2PInt(vec2PInt pVec2PInt)
{
	char ch = 'A';
	for (auto iter1 = pVec2PInt.begin(); iter1 != pVec2PInt.end(); iter1++)
	{
		int i = 0;
		cout << "    robot " << ch << "\n\n";
		for (auto iter2 = iter1->begin(); iter2 != iter1->end(); iter2++, i++)
		{
			cout << "path: " << iter2->first << ", " << iter2->second <<  "\t\t: " << i << "\n";
		}
		cout << "\n";
		ch++;
	}
}

int main()
{
	clock_t startClock, endClock;
	clock_t entire_startClock, entire_endClock;

	entire_startClock = clock();
	for (int i = 0; i < 30; i++)
	{
		startClock = clock();
		vector<p> start = { {1, 2}, {2, 1}, {3, 5}, {4, 1} };
		vector<p> goal = { {18, 17}, {19, 18}, {16, 18}, {15, 19} };
		vector<p> static_obstacle = { {10, 9}, {5, 15} };
		Planner planner(1, 1, static_obstacle);

		planner.set_max_core();
		vec2PInt result = planner.plan(start, goal, 200, 100, false);
		endClock = clock();

		cout << "\ntime: " << endClock - startClock << "  (ms)\n";
	}
	entire_endClock = clock();

	cout << "\ntotal time: " << entire_endClock - entire_startClock << "  (ms)\n";
	// print_vec2PInt(result);

	return 0;
}
