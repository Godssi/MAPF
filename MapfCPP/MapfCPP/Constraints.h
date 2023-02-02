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
    map<int, set<p>>& operator[](Agent agent);

    auto begin();
    auto end();
    friend std::ostream& operator<< (std::ostream& os, const Constraints& Constraints);
}; 

#endif