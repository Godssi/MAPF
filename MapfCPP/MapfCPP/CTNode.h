#ifndef CTNODE_H
#define CTNODE_H

#include "Constraints.h"

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


class CTNode
{
public:
	Constraints constraint;
	map<Agent, vecPInt> solution;
	bool tr = false;
public:
	CTNode(Constraints constraint, map<Agent, vecPInt> solution) : constraint(constraint), solution(solution) {};
	int sic(map<Agent, vecPInt> solution);
	bool __it__(CTNode other);
	friend std::ostream& operator<<(std::ostream& os, const CTNode& cTnode);
};
#endif