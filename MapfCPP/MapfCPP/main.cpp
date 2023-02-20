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

void print_path(vec2PInt pPath)
{
	char ch = 'A';
	cout << "\n";
	for (auto iter1 = pPath.begin(); iter1 != pPath.end(); iter1++)
	{
		int i = 0;
		cout << "\trobot " << ch << "\n\n";
		for (auto iter2 = iter1->begin(); iter2 != iter1->end(); iter2++, i++)
		{
			cout << " path: " << iter2->first << ", " << iter2->second <<  "\t\t: " << i << "\n";
		}
		cout << "\n";
		ch++;
	}
}

void print_map(Planner planner)
{
	Map map = planner.get_aStarPlanner().get_origin_map();
	cout << "\n\t\t\tMAP\n";
	for (auto iter1 = map.begin(); iter1 != map.end(); iter1++)
	{
		cout << "\n\t";
		for (auto iter2 = iter1->begin(); iter2 != iter1->end(); iter2++)
		{
			cout << *iter2 << " ";
		}
	}
	cout << "\n";
}

int main()
{
	//clock_t startClock, endClock;
	//clock_t entire_startClock, entire_endClock;

	//entire_startClock = clock();
	//for (int i = 0; i < 30; i++)
	//{
	//	startClock = clock();
	//  vector<p> start = { {2, 2}, {38, 1} };
	//  vector<p> goal = { {14, 39}, {45, 31} };
	//  vector<p> static_obstacle = { {39, 2} };
	//	Planner planner(1, 1, static_obstacle);

	//	planner.set_max_core();
	//	vec2PInt result = planner.plan(start, goal, 200, 100, false);
	//	endClock = clock();

	//	cout << "\ntime: " << endClock - startClock << "  (ms)\n";
	//}
	//entire_endClock = clock();

	//cout << "\ntotal time: " << entire_endClock - entire_startClock << "  (ms)\n";

	clock_t startClock, endClock;

	startClock = clock();

	//vector<p> start = { {2, 2}, {38, 1} };
	//vector<p> goal = { {14, 39}, {45, 31} };
	vector<p> start = { {2, 2} };
	vector<p> goal = { {14, 39} };
	vector<p> static_obstacle = { {39, 2} };
	Planner planner(1, 1, static_obstacle);

	planner.set_max_core();
	print_map(planner);
	vec2PInt result = planner.plan(start, goal, 200, 100, false);
	print_path(result);

	endClock = clock();
	cout << "\n\n\ttime: " << endClock - startClock << "  (ms)\n";

	return 0;
}
