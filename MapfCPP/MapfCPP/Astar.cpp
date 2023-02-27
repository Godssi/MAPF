#include "Astar.h"

bool cmp(const Node* n1, const Node* n2)
{
	if (n1->f > n2->f)
		return true;
	else
		return false;
}

vector<Node*>::const_iterator findIdx(Node* child, const vector<Node*>& openList)
{
	vector<Node*>::const_iterator iter = openList.begin();
	while (iter != openList.end())
	{
		if (*child == *iter)
			return iter;
		else
			iter++;
	}
	return iter;
}

template<class T>
void deleteVector(vector<T*> pVector)
{
	while (!pVector.empty()) {
		delete pVector.back();
		pVector.pop_back();
	}
}

bool valid_path(pairInt xy, Node * cur, map<int, set<pairInt>> conf_path, bool goal_time = false)
{
	if (goal_time)
	{
		for (auto iter = conf_path.begin(); iter != conf_path.end(); iter++)
		{
			for (auto iter2 = iter->second.begin(); iter2 != iter->second.end(); iter2++)
			{
				if ((xy.first == iter2->first) && (xy.second == iter2->second) && (cur->length + 1 >= iter->first))
				{
					return true;
				}
			}
		}
		return false;
	}
	else
	{
		for (auto iter = conf_path.begin(); iter != conf_path.end(); iter++)
		{
			for (auto iter2 = iter->second.begin(); iter2 != iter->second.end(); iter2++)
			{
				if ((xy.first == iter2->first) && (xy.second == iter2->second) && (cur->length + 1 == iter->first))
				{
					return true;
				}
			}
		}
		return false;
	}
}

bool valid_path2(Node* cur, map<int, set<pairInt>> conf_path, bool goal_time = false)
{
	if (goal_time)
	{
		for (auto iter = conf_path.begin(); iter != conf_path.end(); iter++)
		{
			for (auto iter2 = iter->second.begin(); iter2 != iter->second.end(); iter2++)
			{
				if ((cur->position.first == iter2->first) && (cur->position.second == iter2->second) && (cur->length >= iter->first))
				{
					return false;
				}
			}
		}
		return true;
	}
	else
	{
		for (auto iter = conf_path.begin(); iter != conf_path.end(); iter++)
		{
			for (auto iter2 = iter->second.begin(); iter2 != iter->second.end(); iter2++)
			{
				if ((cur->position.first == iter2->first) && (cur->position.second == iter2->second) && (cur->length == iter->first))
				{
					return false;
				}
			}
		}
		return true;
	}
}


Path AStar(pairInt start, pairInt goal, Map& origin_map, Map& potential_map, map<int, set<pairInt>> conf_path, map<int, set<pairInt>> semi_dynamic_obstacles)
{
	Path pathIdx;
	Node* startNode = new Node(start);
	Node* goalNode = new Node(goal);

	vector<Node*> openList;
	vector<Node*> closedList;

	vector<Node*> tmp;

	openList.push_back(startNode);
	Node* curNode;
	vector<pairInt> dxdy = { {-1, 0}, {0, -1}, {1, 0}, {0, 1} };

	while (!openList.empty())
	{
		sort(openList.begin(), openList.end(), cmp);

		curNode = openList.back();
		openList.pop_back();
		closedList.push_back(curNode);

		if (*curNode == goalNode)
		{
			Node* cur = curNode;
			while (cur->parent != nullptr)
			{
				pairInt pos = cur->position;
				pathIdx.push_back(pos);
				cur = cur->parent;
			}
			pathIdx.push_back(startNode->position);

			deleteVector<Node>(openList);
			deleteVector<Node>(closedList);
			delete goalNode;

			reverse(pathIdx.begin(), pathIdx.end());
			return pathIdx;
		}

		vector<Node*> childrenList;

		for (int i = 0; i < 4; i++)
		{
			pairInt new_xy = { curNode->position.first + dxdy[i].first, curNode->position.second + dxdy[i].second };

			if ((new_xy.first < 0 && new_xy.first >= origin_map.size()) &&
				(new_xy.second < 0 && new_xy.second >= origin_map.front().size()))
				continue;

			if (origin_map[new_xy.first][new_xy.second] != 0)
				continue;

			if (valid_path(new_xy, curNode, conf_path)) continue;

			if (valid_path(new_xy, curNode, semi_dynamic_obstacles, true)) continue;

			Node* newNode = new Node(new_xy, curNode);
			childrenList.push_back(newNode);
			tmp.push_back(newNode);
		}

		while (!childrenList.empty())
		{
			Node* child = childrenList.back();
			childrenList.pop_back();

			if ((*child) ^ closedList)
			{
				delete child;
				continue;
			}

			child->g = curNode->g + 1;
			child->h = heuristic(child->position, goalNode->position, origin_map, potential_map);
			child->f = child->g + child->h;

			if ((*child) ^ (openList))
			{
				auto it = findIdx(child, openList);
				if (it != openList.end() && (child->g < (*it)->g) && valid_path2(child, conf_path) && valid_path2(child, semi_dynamic_obstacles, true))
				{
					delete (*it);
					openList.erase(it);
				}
				else
				{
					delete child;
					continue;
				}
			}
			openList.push_back(child);
		}
	}

	deleteVector<Node>(openList);
	deleteVector<Node>(closedList);
	delete goalNode;

	return pathIdx;
}
