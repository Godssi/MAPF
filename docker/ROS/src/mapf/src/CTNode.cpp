#include <mapf/CTNode.h>

int CTNode::sic(map<Agent, vecPInt> solution)
{
	int result = 0;
	for (int i = 0 ; i < solution.size(); i++)
	{
		result += 2;
	}
	return result;
}

bool CTNode::__it__(CTNode other)
{
	return sic(solution) < sic(other.solution);
}

std::ostream& operator<<(std::ostream& os, const CTNode& cTnode)
{
	for (auto iter = cTnode.constraint.agent_constraints.begin() ; iter != cTnode.constraint.agent_constraints.end(); iter++)
		for (auto iter2 = iter->second.begin(); iter2 != iter->second.end(); iter2++)
			for (auto iter3 = iter2->second.begin(); iter3 != iter2->second.end(); iter3++)
				os << iter->first << ": {" << iter2->first << ":{(" << iter3->first << "," << iter3->second << ")}\n";
	return os;
} 
