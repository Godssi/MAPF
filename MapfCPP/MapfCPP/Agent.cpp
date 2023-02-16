#include "Agent.h"

int Agent::hash() const
{
	return start.first * 1000 + start.second;
}

bool Agent::operator == (const Agent& other) const
{
	if (start == other.start && goal == other.goal)
		return true;
	else
		return false;
}

bool Agent::operator< (const Agent& other) const
{
	return (this->hash() < other.hash());
}

string Agent::str()
{
	string result = "[" + start.first + ', ' + start.second + ']';
	return result;
}
string Agent::repr()
{
	return str();
}

std::ostream& operator<< (std::ostream& os, const Agent& agent)
{
	os << '[' << agent.start.first << "," << agent.start.second << ']';
	return os;
}