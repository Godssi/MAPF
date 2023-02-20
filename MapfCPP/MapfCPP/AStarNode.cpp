#include "AStarNode.h"

bool Node::operator==(const Node other)
{
	if (this->position.first != other.position.first)
		return false;
	else if (this->position.second != other.position.second)
		return false;
	else
		return true;
}

bool Node::operator==(const Node* other)
{
	if (this->position.first != other->position.first)
		return false;
	else if (this->position.second != other->position.second)
		return false;
	else
		return true;
}

bool Node::operator^(const vector<Node*>& list)
{
	vector<Node*>::const_iterator iter = list.begin();
	while (iter != list.end())
	{
		if (*this == *iter)
			return true;
		else
			iter++;
	}
	return false;
}

ostream& operator<<(ostream& os, const Node* node)
{
	os << "(" << node->position.first << ", " << node->position.second << ")";
	return os;
}
