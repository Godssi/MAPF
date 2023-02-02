#ifndef ASSIGNER_H
#define ASSIGNER_H

#include <iostream>
#include <vector>
#include <set>
#include <numeric>
#include <utility>
#include <functional>
#include "Agent.h"
#include "Hungarian.h"

using namespace std;
typedef long long ll;
typedef pair<int, int> p;

vector<Agent> min_cost(vector<p> starts, vector<p> goals);
vector<Agent> greedy_assign(vector<p> starts, vector<p> goals);

#endif