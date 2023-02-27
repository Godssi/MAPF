#ifndef HEURISTIC_FUNC_H
#define HEURISTIC_FUNC_H

#include <iostream>
#include <vector>
#include <string>
#include <utility>

using namespace std;
typedef long long ll;
typedef pair<int, int> pairInt;
typedef vector<pairInt> vecPInt;
typedef vector<vecPInt> vec2PInt;

typedef vector<pairInt> Path;
typedef vector<vector<ll>> Map;
typedef pair<pairInt, double> MapIdx;

double heuristic_m(pairInt node, pairInt goal, double d);
double heuristic_d(pairInt node, pairInt goal, double d1, double d2);
double heuristic_e(pairInt node, pairInt goal);

double get_heuristic_to_goal_sq(pairInt node, pairInt goal, double R, Map& potential_map);
double get_heuristic_to_goal_rect(pairInt node, pairInt goal, double R, Map& potential_map);
double heuristic_around_obstacle(pairInt node, pairInt goal, ll r, double R, Map& map, Map& potential_map);
double heuristic(pairInt node, pairInt goal, Map& map, Map& potential_map);

#endif