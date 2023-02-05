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

    map<Agent, map<int, set<p>>> agent_constraints; // 에이전트별 시간에 따른 못가는 좌표

    Constraints fork(Agent agent, p obstacle, int start, int end);
    // start시간부터 end시간까지 해당 agent는 obstacle 좌표에 갈 수 없다.
    map<int, set<p>>& operator[](Agent agent);

    auto begin();
    auto end();
    friend std::ostream& operator<< (std::ostream& os, const Constraints& Constraints);
}; 

#endif