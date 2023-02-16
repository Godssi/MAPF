#include "AStarAgent.h"

void AStarAgent::renew_position(p cur)
{
	this->cur = cur;
}

void AStarAgent::set_path(Path path)
{
	this->path = path;
}

void AStarAgent::set_astar_path(Map origin_map, Map potential_map, set<p> conf_path, set<p> semi_dynamic_obstacles, string type)
{
	if (type == "renew")
		path = AStar(cur, goal, origin_map, potential_map, conf_path, semi_dynamic_obstacles);
	else if (type == "init")
		path = AStar(start, goal, origin_map, potential_map, conf_path, semi_dynamic_obstacles);
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