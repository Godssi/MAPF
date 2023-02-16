#include "Astar.h"
#include "Node.h"
#include "Heuristic_func.h"

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
void deleteVector(vector<T> pVector)
{
	for (auto iter = pVector.begin(); iter != pVector.end(); iter++)
	{
		delete (*iter);
	}
}

void deleteNode(Node* node)
{
	delete node;
}

Path AStar(p start, p goal, Map map, Map potential_map)
{
	Path pathIdx;
	Node* startNode = new Node(start);
	Node* goalNode = new Node(goal);

	vector<Node*> openList;
	vector<Node*> closedList;

	openList.push_back(startNode);
	Node* curNode;
	vector<p> dxdy = { {-1, 0}, {0, -1}, {1, 0}, {0, 1} };

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
				p pos = cur->position;
				pathIdx.push_back(pos);
				cur = cur->parent;
			}
			deleteVector<Node*>(openList);
			deleteVector<Node*>(closedList);
			delete goalNode;
			return pathIdx;
		}

		vector<Node*> childrenList;

		for (int i = 0; i < 4; i++)
		{
			p new_xy = { curNode->position.first + dxdy[i].first, curNode->position.second + dxdy[i].second };

			if ((new_xy.first < 0 && new_xy.first > map.size() - 1) &&
				(new_xy.second < 0 && new_xy.second > map.front().size() - 1))
				continue;

			if (map[new_xy.first][new_xy.second] == 1 ||
				map[new_xy.first][new_xy.second] == 2 ||
				map[new_xy.first][new_xy.second] == 3)
				continue;

			Node* newNode = new Node(new_xy, curNode);
			childrenList.push_back(newNode);
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
			child->h = heuristic(child->position, goalNode->position, map, potential_map);
			child->f = child->g + child->h;

			if ((*child) ^ (openList))
			{
				auto it = findIdx(child, openList);
				if (child->g < (*it)->g)
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
	return pathIdx;
}