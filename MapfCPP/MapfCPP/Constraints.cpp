#include "Constraints.h"

Constraints Constraints::fork(Agent agent, p obstacle, int start, int end)
{
    map<Agent, map<int, set<p>>> agent_constraints_copy = agent_constraints;
    for (int time = start; time < end; time++) {
        agent_constraints_copy[agent][time].insert(obstacle);
    }
    Constraints new_constraints;
    new_constraints.agent_constraints = agent_constraints_copy;
    return new_constraints;
}

map<int, set<p>>& Constraints::operator[](Agent agent)
{
    return agent_constraints[agent];
}

auto Constraints::begin()
{
    return this->agent_constraints.begin();
}

auto Constraints::end()
{
    return this->agent_constraints.end();
}

std::ostream& operator<< (std::ostream& os, const Constraints& Constraints) // unordered_map<Agent, map<int, set<pair<int, int>>>> agent_constraints_copy = agent_constraints; // {a:{2:{(3,3)}}}
{
    for (auto iter = Constraints.agent_constraints.begin(); iter != Constraints.agent_constraints.end(); iter++)
        for (auto iter2 = iter->second.begin(); iter2 != iter->second.end(); iter2++)
            for (auto iter3 = iter2->second.begin(); iter3 != iter2->second.end(); iter3++)
                os << iter->first << ": {" << iter2->first << ":{(" << iter3->first << "," << iter3->second << ")}\n";
    return os;
}

int main()
{
    Agent a({ 1,5 }, { 3,2 }, "a");
    Agent b({ 1,5 }, { 3,2 }, "b");
    std::vector<std::pair<int, int>> solution1 = { {3,3},{2,3},{3,2} };
    std::vector<std::pair<int, int>> solution2 = { {1,5},{2,3},{4,1} };
    map<Agent, std::vector<pair<int, int>>> sol1;
    std::pair<int, int> obs = { 1,2 };
    sol1[a] = solution1;
    sol1[b] = solution2;
    Constraints con;
    con.fork(a, obs, 1, 4);
    std::cout << con; 
}