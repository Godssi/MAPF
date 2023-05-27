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
typedef vector<pair<vecPInt, int>> dynamicOb;

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
			cout << " path: " << iter2->first << ", " << iter2->second << "\t\t: " << i << "\n";
		}
		cout << "\n";
		ch++;
	}
}

void print_origin_map(Planner planner)
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

void print_static_potential_map(Planner planner)
{
	Map map = planner.get_static_potential_map();
	cout << "\n\t\t\t\tStatic Potential Map\n";
	for (auto iter1 = map.begin(); iter1 != map.end(); iter1++)
	{
		cout << "\n\t";
		for (auto iter2 = iter1->begin(); iter2 != iter1->end(); iter2++)
		{
			cout.width(2);
			cout.fill('0');
			cout << *iter2 << " ";
		}
	}
	cout << "\n";
}

void print_dynamic_potential_map(Planner planner)
{
	Map map = planner.get_aStarPlanner().get_dynamic_potential_map();
	cout << "\n\t\t\t\tDynamic Potential Map\n";
	for (auto iter1 = map.begin(); iter1 != map.end(); iter1++)
	{
		cout << "\n\t";
		for (auto iter2 = iter1->begin(); iter2 != iter1->end(); iter2++)
		{
			cout.width(2);
			cout.fill('0');
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
				// print_path_text에서 \n 으로 좌표 자체를 알기 힘들게 한 것을 조금 더 편하게 좌표처럼 바꾸면 좋을 것 같다.
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

	// 시뮬레이션 상황 처리 
	vector<pairInt> start = { {3, 8}, {7, 32}, {22, 40}, {36, 34}, {59, 45}, {27, 8}, {42, 12}, {81, 13} };
	vector<pairInt> goal = { {17, 17}, {7, 40}, {22, 51}, {37, 46}, {47, 45}, {23, 22}, {37, 33}, {63, 14} };
	vector<pairInt> static_obstacle = { {24, 12}, {10, 20} };
	// 컨베이어 벨트 확인하는 사람의 경로 
	DynamicObstacle DynamicObstacle1("dy_ob1", { {81, 2}, {81, 3}, {81, 4}, {81, 5}, {81, 6}, {81, 7}, {81, 8}, {81, 9}, {81, 10}, {81, 11}, {81, 12}, {81, 13}, {81, 14}, {81, 15}, {81, 16}, {81, 17}, {81, 18}, {81, 19}, {81, 18}, {81, 19}, {81, 20}, {81, 21}, {81, 22}, {81, 23}, {81, 24}, {81, 25}, {81, 26}, {81, 27}, {81, 28}, {81, 29}, {81, 30}, {81, 31}, {81, 32} });
	// 위 물건 창고 확인하는 사람의 경로
	DynamicObstacle DynamicObstacle2("dy_bo2", { {2, 2}, {2, 3}, {2, 4}, {2, 5}, {2, 6}, {2, 7}, {2, 8}, {2, 9}, {2, 10}, {2, 11}, {2, 12}, {2, 13}, {2, 14},{2, 15}, {2, 16}, {2, 17}, {2, 18}, {2, 19}, {2, 20}, {2, 21}, {2, 22}, {2, 23}, {3, 23}, {4, 23}, {5, 23}, {6, 23}, {7, 23}, {8, 23}, {9, 23}, {10, 23}, {11, 23}, {12, 23}, {13, 23}, {14, 23}, {15, 23}, {16, 23}, {17, 23}, {18, 23}, {19, 23}, {20, 23}, {21, 23}, {22, 23}, {23, 23} });
	// 아래 물건 창고 확인하는 사람의 경로
	DynamicObstacle DynamicObstacle3("dy_ob3", { {10, 48}, {11, 48}, {12, 48}, {13, 48}, {14, 48}, {15, 48}, {16, 48}, {17, 48}, {18, 48}, {19, 48}, {20, 48}, {21, 48}, {22, 48}, {23, 48}, {24, 48}, {25, 48}, {26, 48}, {27, 48}, {28, 48}, {29, 48}, {30, 48}, {31, 48}, {32, 48}, {33, 48}, {34, 48}, {35, 48}, {36, 48}, {37, 48}, {38, 48}, {39, 48}, {40, 48}, {41, 48}, {42, 48}, {43, 48}, {44, 48}, {45, 48}, {46, 48}, {47, 48}, {47, 49}, {47, 50} });
	vector<DynamicObstacle> dynamic_obstacle = { DynamicObstacle1, DynamicObstacle2, DynamicObstacle3 }; // 동적 장애물을 직접 만들어서 넣어줌!
	Planner planner(start, goal, 1, 1, static_obstacle, dynamic_obstacle);

	if (planner.validate_agent_position())
	{
		cout << "not valid agent position\n";
		return 0;
	}

	planner.set_max_core();
	vec2PInt result = planner.plan(200, 2000);

	endClock = clock();
	cout << "\n\n\ttime: " << endClock - startClock << "  (ms)\n";

	print_path(result);

	print_origin_map(planner);
	print_static_potential_map(planner);
	print_dynamic_potential_map(planner);
	

	return 0;
}


