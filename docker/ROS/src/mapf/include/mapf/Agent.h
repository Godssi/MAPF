#ifndef AGENT_H
#define AGENT_H

#include <iostream>
#include <utility>
#include <string>
#include <vector>
#include <map>
#include <list>
#include <cmath>
#include <algorithm>
#include <functional>

using namespace std;
typedef pair<int, int> pairInt;

class Agent
{
public:
	pairInt start;
	pairInt goal;
	string agentName;

	Agent(pairInt start, pairInt goal) : start(start), goal(goal) {}
	Agent(pairInt start, pairInt goal, string agentName) : start(start), goal(goal), agentName(agentName) {}
	
	int hash() const;
	string str();
	string repr();
	bool operator == (const Agent& other) const;
	bool operator< (const Agent& other) const;
	friend std::ostream& operator<< (std::ostream& os, const Agent& a);
};



#endif