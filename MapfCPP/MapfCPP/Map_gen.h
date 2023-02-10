#ifndef MAP_GEN_H
#define MAP_GEN_H

#include <iostream>
#include <vector>
#include <string>
#include <utility>

using namespace std;
typedef long long ll;
typedef pair<int, int> p;
typedef vector<p> Path;
typedef vector<vector<ll>> Map;

Map potential_map_generator(Map map);
Map test_maze_gen();
Map random_maze_gen();

# endif