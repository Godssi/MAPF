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
typedef pair<int, int> p;
typedef vector<p> Path;
typedef vector<vector<ll>> Map;

namespace MAP_GEN
{
	Map potential_map_generator(Map map);
	Map modify_potential_map(Map map, Map potential_map);
	Map test_maze_gen();
	Map random_maze_gen();
}

# endif