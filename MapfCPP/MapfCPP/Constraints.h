#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include "Assigner.h"

using namespace std;
typedef pair<int, int> p;

class Constraints
{
public:
    Constraints() {}
    map<Agent, map<int, set<p>>> agent_constraints;
    Constraints fork(Agent agent, p obstacle, int start, int end);
    map<int, set<p>> setdefault(Agent agent, map<int, set<p>> constraint);
    map<int, set<p>> getitem(Agent);
    std::vector<Agent> iter();
    friend std::ostream& operator<< (std::ostream& os, const Constraints& Constraints);
}; 

#endif