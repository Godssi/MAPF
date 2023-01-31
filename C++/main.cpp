#include <iostream>
#include "Agent.h"
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
	Map map = test_maze_gen();
	print_map(map);
	cout << '\n';
	Map potential_map = potential_map_generator(map);
	Agent A({ 1, 1 }, { 1, 1 }, { 20, 20 }, "A");
	
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