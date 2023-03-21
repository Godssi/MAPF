#include "DynamicObstacle.h"

int DynamicObstacle::hash() const
{
	return path[0].first * 1000 + path[0].second;
}

bool DynamicObstacle::operator == (const DynamicObstacle& other) const
{
	if (path[0] == other.path[0] && path[-1] == other.path[-1])
		return true;
	else
		return false;
}

bool DynamicObstacle::operator< (const DynamicObstacle& other) const
{
	return (this->hash() < other.hash());
}


string DynamicObstacle::str()
{
	string result = "[" + path[0].first + ', ' + path[0].second + ']';
	return result;
}

string DynamicObstacle::repr()
{
	return str();
}

std::ostream& operator<< (std::ostream& os, const DynamicObstacle& agent)
{
	os << '[' << agent.path[0].first << "," << agent.path[0].second << ']';
	return os;
}

void DynamicObstacle::Direction(vector< pairInt> path)
{
	pairInt present_pos = path[present_idx];
	pairInt after_pos = path[after_idx];

	if (present_pos == path[-1]) direct_vector = None; // 이런 상황에서는 에러 처리 필요

	if (after_pos.second - present_pos.second == 1)
	{
		switch (after_pos.first - present_pos.first)
		{
		case -1:
			direct_vector = North_West;
			break;
		case 0:
			direct_vector = North;
			break;
		case 1:
			direct_vector = North_East;
			break;
		}
	}
	else if(after_pos.second - present_pos.second == 0)
	{
		switch (after_pos.first - present_pos.first)
		{
		case -1:
			direct_vector = West;
			break;
		case 1:
			direct_vector = East;
			break;
		}
	}
	else
	{
		switch (after_pos.first - present_pos.first)
		{
		case -1:
			direct_vector = South_West;
			break;
		case 0:
			direct_vector = South;
			break;
		case 1:
			direct_vector = South_East;
			break;
		}

		present_idx = after_idx;
		after_idx = after_idx + 1;
	}
}