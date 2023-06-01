#include <iostream>

#include "AStar.h"
#include "AStarMapGen.h"
#include "PathAnalyzer.h"
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


void print_potential_map(Planner planner)
{
	Map map1 = planner.get_static_potential_map();
	Map map2 = planner.get_aStarPlanner().get_dynamic_potential_map();
	cout << "\n\t\t\t\tPotential Map\n";

	for (int i = 0; i < map1.size(); i++)
	{
		for (int j = 0; j < map1[0].size(); j++)
		{
			map1[i][j] += map2[i][j];
		}
	}

	for (auto iter1 = map1.begin(); iter1 != map1.end(); iter1++)
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



int main()
{
	clock_t startClock, endClock;


	// 시뮬레이션 상황 처리 
	//vector<pairInt> start = { {16,83},{35,83},{29,66},{10,66}, {3,10},{48,8},{48,34},{50,66} };
	//vector<pairInt> goal = { {40, 3},{3,63},{20,15},{48,20},{57,50},{8,50},{13,6},{35,4} };

	//vector<pairInt> static_obstacle;
	//
	//DynamicObstacle DynamicObstacle1("dy_ob1");
	//dynamic_path_input_y(DynamicObstacle1, 48, 19, 48, 29);
	//dynamic_path_input_x(DynamicObstacle1, 47, 29, 39, 29);
	//dynamic_path_input_y(DynamicObstacle1, 39, 30, 39, 43);
	//dynamic_path_input_x(DynamicObstacle1, 38, 43, 14, 43);
	//dynamic_path_input_y(DynamicObstacle1, 14, 42, 14, 12);
	//dynamic_path_input_x(DynamicObstacle1, 13, 12, 1, 12);
	//DynamicObstacle1.Direction();

	//DynamicObstacle DynamicObstacle2("dy_ob2");
	//dynamic_path_input_x(DynamicObstacle2, 13, 58, 16, 58);
	//dynamic_path_input_y(DynamicObstacle2, 16, 57, 16, 30);
	//dynamic_path_input_x(DynamicObstacle2, 17, 30, 35, 30);
	//dynamic_path_input_y(DynamicObstacle2, 35, 29, 35, 13);
	//DynamicObstacle2.Direction();

	//DynamicObstacle DynamicObstacle3("dy_ob3");
	//dynamic_path_input_x(DynamicObstacle3, 10, 81, 51, 81);
	//dynamic_path_input_y(DynamicObstacle3, 51, 80, 51, 64);
	//DynamicObstacle3.Direction();

	//vector<DynamicObstacle> dynamic_obstacle = { DynamicObstacle1, DynamicObstacle2, DynamicObstacle3 }; // 동적 장애물을 직접 만들어서 넣어줌!
	//Planner planner(start, goal, 1, 1, static_obstacle, dynamic_obstacle);
	//planner.set_max_core();

	//if (planner.validate_agent_position())
	//{
	//	cout << "not valid agent position\n";
	//	return 0;
	//}
	//vec2PInt result;
	//startClock = clock();
	//for (int i = 0; i < 1; i++)
	//	result = planner.plan(200, 100000);
	//endClock = clock();

	////print_potential_map(planner);
	//cout << "time: " << (endClock - startClock) / 10.0 << "  (ms)\n";
	//getPathLength(result);
	//nearestDistance2Obstacle(result);
	//averageDistance2Obstacle(result);
	//// print_path(result);
	//print_path_text(result);
	//


	//cout << "\n\n\n";

	//DynamicObstacle DynamicObstacle4("dy_ob1");
	//dynamic_path_input_y(DynamicObstacle1, 48, 19, 48, 29);
	//dynamic_path_input_x(DynamicObstacle1, 47, 29, 39, 29);
	//dynamic_path_input_y(DynamicObstacle1, 39, 30, 39, 43);
	//dynamic_path_input_x(DynamicObstacle1, 38, 43, 14, 43);
	//dynamic_path_input_y(DynamicObstacle1, 14, 42, 14, 12);
	//dynamic_path_input_x(DynamicObstacle1, 13, 12, 1, 12);
	//DynamicObstacle1.Direction();

	//DynamicObstacle DynamicObstacle5("dy_ob2");
	//dynamic_path_input_x(DynamicObstacle2, 13, 58, 16, 58);
	//dynamic_path_input_y(DynamicObstacle2, 16, 57, 16, 30);
	//dynamic_path_input_x(DynamicObstacle2, 17, 30, 35, 30);
	//dynamic_path_input_y(DynamicObstacle2, 35, 29, 35, 13);
	//DynamicObstacle2.Direction();

	//DynamicObstacle DynamicObstacle6("dy_ob3");
	//dynamic_path_input_x(DynamicObstacle3, 10, 81, 51, 81);
	//dynamic_path_input_y(DynamicObstacle3, 51, 80, 51, 64);
	//DynamicObstacle3.Direction();

	//vector<DynamicObstacle> dynamic_obstacle2 = { DynamicObstacle4, DynamicObstacle5, DynamicObstacle6 }; // 동적 장애물을 직접 만들어서 넣어줌!

	//Planner planner2(start, goal, 1, 1, static_obstacle, dynamic_obstacle2);
	//planner2.set_max_core();
	//planner2.aStarPlanner.set_origin_path(true);

	//if (planner2.validate_agent_position())
	//{
	//	cout << "not valid agent position\n";
	//	return 0;
	//}
	//startClock = clock();
	//for (int i = 0; i < 1; i++)
	//	result = planner2.plan(200, 100000);
	//endClock = clock();

	//// print_potential_map(planner);
	//cout << "time: " << (endClock - startClock) / 10.0 << "  (ms)\n";
	//getPathLength(result);
	//nearestDistance2Obstacle(result);
	//averageDistance2Obstacle(result);
	//// print_path_text(result);



	// 시뮬레이션 상황 처리
	vec3PInt results;

	vector<pairInt> start = { {16 ,83}, {35, 83}, {29, 66}, {10, 66}, {3, 10}, {48, 8}, {48, 34}, {50, 66} };
	vector<pairInt> goal = { {40, 3}, {3, 63}, {20, 15}, {48, 20}, {57, 50}, {8, 50}, {13, 6}, {35, 4} };

	vector<pairInt> static_obstacle;
	//DynamicObstacle DynamicObstacle1("dy_ob1");
	//dynamic_path_input_y(DynamicObstacle1, 48, 19, 48, 29);
	//dynamic_path_input_x(DynamicObstacle1, 47, 29, 39, 29);
	//dynamic_path_input_y(DynamicObstacle1, 39, 30, 39, 43);
	//dynamic_path_input_x(DynamicObstacle1, 38, 43, 14, 43);
	//dynamic_path_input_y(DynamicObstacle1, 14, 42, 14, 12);
	//dynamic_path_input_x(DynamicObstacle1, 13, 12, 1, 12);
	//DynamicObstacle1.Direction();

	DynamicObstacle DynamicObstacle2("dy_ob2");
	dynamic_path_input_x(DynamicObstacle2, 13, 58, 16, 58);
	dynamic_path_input_y(DynamicObstacle2, 16, 57, 16, 30);
	dynamic_path_input_x(DynamicObstacle2, 17, 30, 35, 30);
	dynamic_path_input_y(DynamicObstacle2, 35, 29, 35, 13);
	DynamicObstacle2.Direction();

	DynamicObstacle DynamicObstacle3("dy_ob3");
	dynamic_path_input_x(DynamicObstacle3, 10, 81, 51, 81);
	dynamic_path_input_y(DynamicObstacle3, 51, 80, 51, 64);
	DynamicObstacle3.Direction();

	//vector<DynamicObstacle> dynamic_obstacle = { DynamicObstacle1, DynamicObstacle2, DynamicObstacle3 };
	//vector<DynamicObstacle> dynamic_obstacle_valid = { DynamicObstacle1, DynamicObstacle2, DynamicObstacle3 };
	vector<DynamicObstacle> dynamic_obstacle = { DynamicObstacle2, DynamicObstacle3 };
	vector<DynamicObstacle> dynamic_obstacle_valid = { DynamicObstacle2, DynamicObstacle3 };
	Planner planner(start, goal, 1, 1, static_obstacle, dynamic_obstacle);
	planner.set_max_core();

	if (planner.validate_agent_position())
	{
		cout << "not valid agent position\n";
		return 0;
	}

	startClock = clock();
	while (DynamicObstacle2.DoB_path.size() || DynamicObstacle3.DoB_path.size())
	{
		vec2PInt result = planner.plan(200, 10000);

		results.push_back(result);

		// print_path(result);
		if (!(planner.checkGoal(result)))
			break;
		planner.set_starts(result);
		endClock = clock();
		cout << "\n\n\ttime: " << endClock - startClock << "  (ms)\n";
	}

	nearestDistance2Obstacle(results, dynamic_obstacle_valid);

	return 0;
}


