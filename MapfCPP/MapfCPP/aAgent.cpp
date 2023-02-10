#include "aAgent.h"

void aAgent::renew_position(p cur)
{
	this->cur = cur;
}

void aAgent::set_path(Path path)
{
	this->path = path;
}

void aAgent::set_astar_path(Map map, Map potential_map, string type)
{
	if (type == "renew")
		path = AStar(cur, goal, map, potential_map);
	else if (type == "init")
		path = AStar(start, goal, map, potential_map);
}

bool aAgent::move_path()
{
	if (path.size() > 1)
	{
		cur = path[path.size() - 1];
		return true;
	}
	cur = path[0];
	return false;
}

void aAgent::print_position()
{
	cout << "aAgent " << this->name << ": ( " << this->cur.first << ", " << this->cur.second << " )" << endl;
}

void aAgent::set_position()
{
	this->position.push_back(this->cur);
}