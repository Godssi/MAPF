#ifndef ASTAR_MAP_GEN_H
#define ASTAR_MAP_GEN_H
#define PI 3.141592

#include <iostream>
#include <vector>
#include <string>
#include <utility>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include "DynamicObstacle.h"

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
/*typedef vector<DynamicObstacle> dynamicOb*/; // Dynamic Object�� ���� ���� ������ �� �ֱ� ������ 

namespace MAP_GEN
{
	Map potential_map_generator(const Map& map);
	Map dynamic_potential_map(const Map& map, const vector<DynamicObstacle>& dynamic_obstacles);
	Map moving_obstacle_to_origin_map(Map map, const vecPInt& movePoint);
	Map test_maze_gen();

	bool Ellipse_equation(int x, int y, int search_x, int search_y, int direct, int speed, char Big_Small);
	double cosine_degree(int degree);
	double sine_degree(int degree);
}

# endif