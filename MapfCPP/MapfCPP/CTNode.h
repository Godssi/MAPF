#ifndef CTNODE_H
#define CTNODE_H

#include "Constraints.h"

class CTNode
{
public:
	Constraints constraint;
	map<Agent, std::vector<std::pair<int, int>>> solution;
public:
	CTNode(Constraints constraint, map<Agent, std::vector<std::pair<int, int>>> solution) : constraint(constraint), solution(solution) {};

	int sic(map<Agent, std::vector<std::pair<int, int>>> solution);
	bool __it__(CTNode other);
	friend std::ostream& operator<<(std::ostream& os, const CTNode& cTnode);
};
#endif