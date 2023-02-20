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
typedef pair<int, int> pairInt;
typedef pair<Agent, Agent> pairAgent;
typedef vector<Agent> vecAgent;
typedef vector<pairInt> vecPInt;
typedef vector<pairAgent> vecPAgent;
typedef vector<vecPInt> vec2PInt;
typedef set<pairInt> setPInt;


vecAgent min_cost(vecPInt starts, vecPInt goals);
vecAgent greedy_assign(vecPInt starts, vecPInt goals);
vecAgent _assign(vecPInt starts, vecPInt goals);


#endif