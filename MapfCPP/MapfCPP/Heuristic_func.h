#ifndef HEURISTIC_FUNC_H
#define HEURISTIC_FUNC_H

#include <iostream>
#include <vector>
#include <string>
#include <utility>

using namespace std;
typedef long long ll;
typedef pair<int, int> p;
typedef vector<p> Path;
typedef vector<vector<ll>> Map;
typedef pair<p, double> MapIdx;

double heuristic_m(p node, p goal, double d);
double heuristic_d(p node, p goal, double d1, double d2);
double heuristic_e(p node, p goal);

vector<p> get_around_index(p node, double r, Map map);
vector<MapIdx> get_index_to_goal_sq(p node, p goal);
pair<vector<MapIdx>, vector<MapIdx>> get_index_to_goal_rect(p node, p goal);
double heuristic_around_obstacle(p node, p goal, Map map, Map potential_map);
double heuristic(p node, p goal, Map map, Map potential_map);

#endif