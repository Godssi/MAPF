#ifndef ASTAR_AGENT_H
#define ASTAR_AGENT_H

#include <iostream>
#include <vector>
#include <string>
#include <utility>

#include "AStar.h"

using namespace std;
typedef long long ll;
typedef pair<int, int> p;
typedef vector<p> Path;
typedef vector<vector<ll>> Map;

class AStarAgent
{
public:
    p start;
    p cur;
    p goal;
    string name;
    Path path;
    ll aTime = 0;
    vector<p> position;
public:
    AStarAgent(p start, p cur, p goal, string name = "None")
        : start(start), cur(cur), goal(goal), name(name) {}
    void renew_position(p cur);
    void set_path(Path path);
    void set_astar_path(Map map, Map potential_map, string type);
    bool move_path();
    void print_position();
    void set_position();
    void aTimePlus() { aTime++; }
};

# endif