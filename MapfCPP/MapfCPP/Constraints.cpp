#include "Constraints.h"

// 단순하게 충돌이 일어나는 시간과 constriant의 위치를 deep copy 하는 코드
Constraints Constraints::fork(Agent agent, pairInt obstacle, int start, int end) // calculate_constraints에 의해 호출됨 (agent1, (8,6), 10 ,12)
{
    map<Agent, map<int, setPInt>> agent_constraints_copy = agent_constraints;
    // agent_constraints에 아무것도 안들어가있음. 
    for (int time = start; time < end; time++) {
        agent_constraints_copy[agent][time].insert(obstacle); // 10, 11 인덱스에 (8,6) 넣어줌
    }
    Constraints new_constraints;
    new_constraints.agent_constraints = agent_constraints_copy;
    return new_constraints; // 10, 11초에 (8,6)에서 충돌이 일어남을 리턴함
}


// Agent 별롤 충돌이 일어나는 시간과 충돌이 일어나는 위치를 넣어주는 것
map<int, setPInt> Constraints::setdefault(Agent agent, map<int, setPInt> constraint)
{
    if (agent_constraints.find(agent) != agent_constraints.end())
    {
        return agent_constraints[agent];
    }
    else
    {
        agent_constraints.insert({ agent, constraint });
        return agent_constraints[agent];
    }
}


// agent별로의 시간에 따른 constraint를 반환
map<int, setPInt> Constraints::getitem(Agent agent)
{
    return this->agent_constraints[agent];
}


// 전체 constriant에 저장되어있는 Agent들을 반환
vecAgent Constraints::iter()
{
    vecAgent agent_list;
    for (auto constraint : agent_constraints)
        agent_list.push_back(constraint.first);
    return agent_list;
}


// 단순하게 agent_constraint를 string형으로 반환하는 코드
std::ostream& operator<< (std::ostream& os, const Constraints& Constraints) // unordered_map<Agent, map<int, set<pair<int, int>>>> agent_constraints_copy = agent_constraints; // {a:{2:{(3,3)}}}
{
    for (auto iter = Constraints.agent_constraints.begin(); iter != Constraints.agent_constraints.end(); iter++)
        for (auto iter2 = iter->second.begin(); iter2 != iter->second.end(); iter2++)
            for (auto iter3 = iter2->second.begin(); iter3 != iter2->second.end(); iter3++)
                os << iter->first << ": {" << iter2->first << ":{(" << iter3->first << "," << iter3->second << ")}\n";
    return os;
}

