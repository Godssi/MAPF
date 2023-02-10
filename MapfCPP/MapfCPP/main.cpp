#include "Agent.h"
#include "CTNode.h"
#include "Constraints.h"
#include "Assigner.h"
#include "Hungarian.h"

#include <iostream>
#include "aAgent.h"
#include "Astar.h"
#include "Map_gen.h"

void print_map(Map map)
{
	for (int i = 0; i < map.size(); i++)
	{
		for (int j = 0; j < map.front().size(); j++)
			cout << map[i][j] << ' ';
		cout << '\n';
	}
}

int main()
{
	//Agent a({ 1,5 }, { 3,2 }, "a");
	//Agent b({ 1,5 }, { 3,2 }, "b");
	//std::vector<std::pair<int, int>> solution1 = { {3,3},{2,3},{3,2} };
	//std::vector<std::pair<int, int>> solution2 = { {1,5},{2,3},{4,1} };
	//map<Agent, std::vector<pair<int, int>>> sol1;
	//sol1[a] = solution1;
	//sol1[b] = solution2;
	//Constraints con;
	//CTNode c(con, sol1);
	//std::cout << c;
	Map map = test_maze_gen();
	print_map(map);
	cout << '\n';
	Map potential_map = potential_map_generator(map);
	aAgent A({ 1, 1 }, { 1, 1 }, { 20, 20 }, "A");

	A.set_astar_path(map, potential_map, "init");
	A.print_position();

	bool flag = true;
	while (flag)
	{
		A.set_astar_path(map, potential_map, "renew");
		flag = A.move_path();
		A.set_position();
		A.print_position();
		A.aTimePlus();
	}
	return 0;

}