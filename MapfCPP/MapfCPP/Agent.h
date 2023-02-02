
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
typedef pair<int, int> p;

class Agent
{
public:
	p start;
	p goal;
	string agentName;

	Agent(p start, p goal) : start(start), goal(goal) {}
	Agent(p start, p goal, string agentName) : start(start), goal(goal), agentName(agentName) {}
	
	int hash() const;
	string str();
	string repr();
	bool operator == (const Agent& other) const;
	bool operator< (const Agent& other) const;
	friend std::ostream& operator<< (std::ostream& os, const Agent& a);
};

#endif