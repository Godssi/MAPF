#ifndef PATHANALYZER_H
#define PATHANALYZER_H

#include <iostream>
#include <vector>
#include <set>
#include <numeric>
#include "Agent.h"
#include "CTNode.h"
#include "AStarMapGen.h"

#define MAX 2147483646.0

using namespace std;

typedef long long ll;
typedef pair<int, int> pairInt;
typedef pair<CTNode, CTNode> pairCTNode;
typedef pair<Agent, Agent> pairAgent;
typedef vector<Agent> vecAgent;
typedef vector<CTNode> vecCTNode;
typedef vector<pairInt> vecPInt;
typedef vector<pairAgent> vecPAgent;
typedef vector<vecPInt> vec2PInt;
typedef set<pairInt> setPInt;
typedef vector<pair<vecPInt, int>> dynamicOb;
typedef vector<vector<ll>> Map;

void getPathLength(vec2PInt result);
void nearestDistance2Obstacle(vec2PInt result);
void averageDistance2Obstacle(vec2PInt result);

# endif