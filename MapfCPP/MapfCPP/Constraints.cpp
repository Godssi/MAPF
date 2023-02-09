#include "Constraints.h"

// �ܼ��ϰ� �浹�� �Ͼ�� �ð��� constriant�� ��ġ�� deep copy �ϴ� �ڵ�
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

// Agent ���� �浹�� �Ͼ�� �ð��� �浹�� �Ͼ�� ��ġ�� �־��ִ� ��
std::map<int, std::set<p>> Constraints::setdefault(Agent agent, map<int, set<p>> constraint)
{
    if (agent_constraints.find(agent) != agent_constraints.end())
        return agent_constraints[agent];
    else
    {
        agent_constraints.insert({ agent, constraint });
        return agent_constraints[agent];
    }
}

// agent������ �ð��� ���� constraint�� ��ȯ
map<int, set<p>> Constraints::getitem(Agent agent)
{
    return this->agent_constraints[agent];
}

// ��ü constriant�� ����Ǿ��ִ� Agent���� ��ȯ
std::vector<Agent> Constraints::iter()
{
    std::vector<Agent> agent_list;
    for ( auto constraint : agent_constraints)
        agent_list.push_back(constraint.first);
    return agent_list;
}

// �ܼ��ϰ� agent_constraint�� string������ ��ȯ�ϴ� �ڵ�
std::ostream& operator<< (std::ostream& os, const Constraints& Constraints) // unordered_map<Agent, map<int, set<pair<int, int>>>> agent_constraints_copy = agent_constraints; // {a:{2:{(3,3)}}}
{
    for (auto iter = Constraints.agent_constraints.begin(); iter != Constraints.agent_constraints.end(); iter++)
        for (auto iter2 = iter->second.begin(); iter2 != iter->second.end(); iter2++)
            for (auto iter3 = iter2->second.begin(); iter3 != iter2->second.end(); iter3++)
                os << iter->first << ": {" << iter2->first << ":{(" << iter3->first << "," << iter3->second << ")}\n";
    return os;
}

