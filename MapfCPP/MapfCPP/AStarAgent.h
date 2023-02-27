#ifndef ASTAR_AGENT_H
#define ASTAR_AGENT_H

#include <iostream>
#include <vector>
#include <string>
#include <utility>
#include <map>
#include <set>

#include "AStar.h"

using namespace std;
typedef long long ll;
typedef pair<int, int> pairInt;
typedef vector<pairInt> vecPInt;
typedef vector<vecPInt> vec2PInt;

typedef vector<pairInt> Path;
typedef vector<vector<ll>> Map;

class AStarAgent
{
public:
    pairInt start;
    pairInt cur;
    pairInt goal;
    string name;
    Path path;
    ll aTime = 0;
    vector<pairInt> position;
public:
    AStarAgent(pairInt start, pairInt cur, pairInt goal, string name = "None")
        : start(start), cur(cur), goal(goal), name(name) {}
    void renew_position(pairInt cur);
    void set_path(Path path);
    void set_astar_path(Map origin_map, Map potential_map, map<int, set<pairInt>> conf_path, map<int, set<pairInt>> semi_dynamic_obstacles, string type);
    bool move_path();
    void print_position();
    void set_position();
    void aTimePlus() { aTime++; }

    static Path get_astar_path(pairInt start, pairInt goal, Map& origin_map, Map& potential_map, map<int, set<pairInt>> conf_path, map<int, set<pairInt>> semi_dynamic_obstacles);
};

# endif