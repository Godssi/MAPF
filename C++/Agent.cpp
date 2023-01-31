#include "Agent.h"

void Agent::renew_position(p cur)
{
	this->cur = cur;
}

void Agent::set_path(Path path)
{
	this->path = path;
}

void Agent::set_astar_path(Map map, Map potential_map, string type)
{
	if (type == "renew")
		path = AStar(cur, goal, map, potential_map);
	else if (type == "init")
		path = AStar(start, goal, map, potential_map);
}

bool Agent::move_path()
{
	if (path.size() > 1)
	{
		cur = path[path.size() - 1];
		return true;
	}
	cur = path[0];
	return false;
}

void Agent::print_position()
{
	cout << "Agent " << this->name << ": ( " << this->cur.first << ", " << this->cur.second << " )" << endl;
}

void Agent::set_position()
{
	this->position.push_back({ this->cur, this->aTime });
}