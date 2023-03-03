#include <iostream>

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
	cout << "\n\t\t\t\tMap\n";
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

#include <fstream>

void print_path_text(vec2PInt pPath)
{
	ofstream fout("Visualization/multiagentpath.txt");
	
	if (!pPath.empty())
	{
		fout << pPath.begin()->size();
		for (auto iter1 = pPath.begin(); iter1 != pPath.end(); iter1++)
		{
			for (auto iter2 = iter1->begin(); iter2 != iter1->end(); iter2++)
			{
				fout << "\n" << iter2->first << "\n" << iter2->second;
			}
		}
	}
	else
	{
		fout << 0;
	}
	fout.close();
}

int main()
{
	clock_t startClock, endClock;

	startClock = clock();
	for (int i = 0; i < 1; i++)
	{
		vector<pairInt> start = { {2, 2}, {38, 1}, {2, 32}, {48, 39}, {2, 16}, {2, 1}, {1, 34}, {45, 1} };
		vector<pairInt> goal = { {14, 39}, {45, 31}, {49, 3}, {22, 5}, {24, 2}, {14, 40}, {49, 17}, {17, 1} };
		vector<pairInt> static_obstacle = { {24, 12} };
		Planner planner(start, goal, 1, 1, static_obstacle);

		if (planner.validate_agent_position())
		{
			cout << "not valid agent position\n";
			return 0;
		}

		planner.set_max_core();
		vec2PInt result = planner.plan(200, 1000, false);
	}
	endClock = clock();
	cout << "\n\n\ttime: " << endClock - startClock << "  (ms)\n";

	return 0;
}
