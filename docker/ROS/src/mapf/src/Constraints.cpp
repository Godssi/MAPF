#include <mapf/Constraints.h>

Constraints Constraints::fork(Agent agent, pairInt obstacle, int start, int end)
{
    map<Agent, map<int, setPInt>> agent_constraints_copy = agent_constraints;
    for (int time = start; time < end; time++) {
        agent_constraints_copy[agent][time].insert(obstacle);
    }
    Constraints new_constraints;
    new_constraints.agent_constraints = agent_constraints_copy;
    return new_constraints;
}

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

map<int, setPInt> Constraints::getitem(Agent agent)
{
    return this->agent_constraints[agent];
}

vecAgent Constraints::iter()
{
    vecAgent agent_list;
    for (auto constraint : agent_constraints)
        agent_list.push_back(constraint.first);
    return agent_list;
}

std::ostream& operator<< (std::ostream& os, const Constraints& Constraints)
{
    for (auto iter = Constraints.agent_constraints.begin(); iter != Constraints.agent_constraints.end(); iter++)
        for (auto iter2 = iter->second.begin(); iter2 != iter->second.end(); iter2++)
            for (auto iter3 = iter2->second.begin(); iter3 != iter2->second.end(); iter3++)
                os << iter->first << ": {" << iter2->first << ":{(" << iter3->first << "," << iter3->second << ")}\n";
    return os;
}
