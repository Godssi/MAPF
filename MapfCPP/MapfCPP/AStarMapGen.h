#ifndef ASTAR_MAP_GEN_H
#define ASTAR_MAP_GEN_H

#include <iostream>
#include <vector>
#include <string>
#include <utility>
#include <cstdlib>
#include <ctime>
#include <cmath>

using namespace std;
typedef long long ll;
typedef pair<int, int> pairInt;
typedef vector<pairInt> vecPInt;
typedef vector<vecPInt> vec2PInt;

typedef vector<pairInt> Path;
typedef vector<vector<ll>> Map;

namespace MAP_GEN
{
	Map potential_map_generator(Map map);
	void modify_potential_map(Map map, Map& potential_map);
	Map moving_obstacle_to_origin_map(Map map, const vecPInt& movePoint);
	Map test_maze_gen1();
	Map test_maze_gen2();
	Map random_maze_gen();
}

# endif