#ifndef ASTAR_MAP_GEN_H
#define ASTAR_MAP_GEN_H

#include <iostream>
#include <vector>
#include <string>
#include <utility>
#include <cstdlib>
#include <ctime>
#include <cmath>

enum Object
{
	Ground,
	Outer_Wall,
	Inner_Wall,
	Static_Ob,
	Dynamic_Ob,
};

using namespace std;
typedef long long ll;
typedef pair<int, int> pairInt;
typedef vector<pairInt> vecPInt;
typedef vector<vecPInt> vec2PInt;

typedef vector<pairInt> Path;
typedef vector<vector<ll>> Map;
typedef vector<pair<vecPInt, int>> dynamicOb;

namespace MAP_GEN
{
	Map potential_map_generator(const Map& map);
	Map dynamic_potential_map(const Map& map, const dynamicOb& dynamic_obstacle);
	Map moving_obstacle_to_origin_map(Map map, const vecPInt& movePoint);
	Map test_maze_gen();
}

# endif