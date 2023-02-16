#include "AStarAgent.h"

void AStarAgent::renew_position(p cur)
{
	this->cur = cur;
}

void AStarAgent::set_path(Path path)
{
	this->path = path;
}

void AStarAgent::set_astar_path(Map map, Map potential_map, string type)
{
	if (type == "renew")
		path = AStar(cur, goal, map, potential_map);
	else if (type == "init")
		path = AStar(start, goal, map, potential_map);
}

bool AStarAgent::move_path()
{
	if (path.size() > 1)
	{
		cur = path.back();
		return true;
	}
	cur = path[0];
	return false;
}

void AStarAgent::print_position()
{
	cout << "Agent " << this->name << ": ( " << this->cur.first << ", " << this->cur.second << " )" << endl;
}

void AStarAgent::set_position()
{
	this->position.push_back(this->cur);
}