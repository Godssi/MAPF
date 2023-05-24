﻿#include "DynamicObstacle.h"

int DynamicObstacle::hash() const
{
	return DoB_path[0].first * 1000 + DoB_path[0].second;
}

bool DynamicObstacle::operator == (const DynamicObstacle& other) const
{
	if (DoB_path[0] == other.DoB_path[0] && DoB_path[-1] == other.DoB_path[-1])
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
	string result = "[" + DoB_path[0].first + ', ' + DoB_path[0].second + ']';
	return result;
}

string DynamicObstacle::repr()
{
	return str();
}

std::ostream& operator<< (std::ostream& os, const DynamicObstacle& agent)
{
	os << '[' << agent.DoB_path[0].first << "," << agent.DoB_path[0].second << ']';
	return os;
}

// path�� ��ġ�� ���� direction_vector ���� �ٲپ �־��ִ� �ڵ� 
void DynamicObstacle::Direction(vector<pairInt> path)
{
	while (true)
	{
		pairInt present_pos = path[present_idx];

		if (present_pos == *(path.end() - 1))
		{
			direct_vector.push_back(None); // ������ ��ġ������ ���ư� ������ �����Ƿ� None���� ó��
			break;
		}

		pairInt after_pos = path[after_idx];

		if (after_pos.second - present_pos.second == 1)
		{
			switch (after_pos.first - present_pos.first)
			{
			case -1:
				direct_vector.push_back(North_West);
				break;
			case 0:
				direct_vector.push_back(North);
				break;
			case 1:
				direct_vector.push_back(North_East);
				break;
			}
		}
		else if (after_pos.second - present_pos.second == 0)
		{
			switch (after_pos.first - present_pos.first)
			{
			case -1:
				direct_vector.push_back(West);
				break;
			case 1:
				direct_vector.push_back(East);
				break;
			}
		}
		else
		{
			switch (after_pos.first - present_pos.first)
			{
			case -1:
				direct_vector.push_back(South_West);
				break;
			case 0:
				direct_vector.push_back(South);
				break;
			case 1:
				direct_vector.push_back(South_East);
				break;
			}
		}
		present_idx = after_idx;
		after_idx = after_idx + 1;
	}
}

