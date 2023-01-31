#include "Heuristic_func.h"
#include <tuple>
#include <cmath>

double heuristic_m(p node, p goal, double d = 1)
{
	int dx = abs(node.first - goal.first);
	int dy = abs(node.second - goal.second);
	return d * (dx + dy);
}

double heuristic_d(p node, p goal, double d1 = 1, double d2 = sqrt(2))
{
	int dx = abs(node.first - goal.first);
	int dy = abs(node.second - goal.second);
	return d1 * (dx + dy) + (d2 - 2 * d1) * min(dx, dy);
}

double heuristic_e(p node, p goal)
{
	int dx = abs(node.first - goal.first);
	int dy = abs(node.second - goal.second);
	return sqrt(dx * dx + dy * dy);
}

vector<p> get_around_index(p node, double r, Map map)
{
	pair<ll, ll> map_size = { map.size() , map[0].size() };
	vector<p> idx;
	for (ll i = map_size.first - r; i < map_size.first + 1; i++)
	{
		for (ll j = map_size.second - r; j < map_size.second + 1; j++)
		{
			double dist = sqrt(pow(i - map_size.first, 2) + pow(j - map_size.second, 2));
			if ((0 <= i && i < map_size.first) && (0 <= j && j < map_size.second) && (r >= dist))
			{
				idx.push_back({ i, j });
			}
		}
	}
	return idx;
}

vector<MapIdx> get_index_to_goal_sq(p node, p goal)
{
	vector<MapIdx> idx;
	for (int i = 0; i < goal.first - node.first; i++)
		idx.push_back({ { i + node.first, i + node.second }, sqrt(2) * (i + node.first) });
	return idx;
}

pair<vector<MapIdx>, vector<MapIdx>> get_index_to_goal_rect(p node, p goal)
{
	vector<pair<p, double>> upperIdx;
	vector<pair<p, double>> lowerIdx;
	if (node.first <= goal.first and node.second <= goal.second)
	{
		for (int i = node.first; i < goal.first + 1; i++)
		{
			for (int j = node.second; j < goal.second + 1; j++)
			{
				if ((goal.second - node.second + 1) * (i - goal.first) >= (goal.first - node.first + 1) * (j - goal.second))
				{
					double r = sqrt(pow(i - node.first, 2) + pow(j - node.second, 2));
					upperIdx.push_back({ {i, j}, r });
				}
				if ((goal.second - node.second + 1) * (i - goal.first) <= (goal.first - node.first + 1) * (j - goal.second))
				{
					double r = sqrt(pow(i - node.first, 2) + pow(j - node.second, 2));
					lowerIdx.push_back({ {i, j}, r });
				}
			}
		}
	}
	else if (node.first >= goal.first and node.second <= goal.second)
	{
		for (int i = node.first; i > goal.first - 1; i--)
		{
			for (int j = node.second; j < goal.second + 1; j++)
			{
				if ((goal.second - node.second + 1) * (i - goal.first) >= (goal.first - node.first - 1) * (j - goal.second))
				{
					double r = sqrt(pow(i - node.first, 2) + pow(j - node.second, 2));
					upperIdx.push_back({ {i, j}, r });
				}
				if ((goal.second - node.second + 1) * (i - goal.first) <= (goal.first - node.first - 1) * (j - goal.second))
				{
					double r = sqrt(pow(i - node.first, 2) + pow(j - node.second, 2));
					lowerIdx.push_back({ {i, j}, r });
				}
			}
		}
	}
	else if (node.first >= goal.first and node.second >= goal.second)
	{
		for (int i = node.first; i > goal.first - 1; i--)
		{
			for (int j = node.second; j > goal.second - 1; j--)
			{
				if ((goal.second - node.second - 1) * (i - goal.first) >= (goal.first - node.first - 1) * (j - goal.second))
				{
					double r = sqrt(pow(i - node.first, 2) + pow(j - node.second, 2));
					upperIdx.push_back({ {i, j}, r });
				}
				if ((goal.second - node.second - 1) * (i - goal.first) <= (goal.first - node.first - 1) * (j - goal.second))
				{
					double r = sqrt(pow(i - node.first, 2) + pow(j - node.second, 2));
					lowerIdx.push_back({ {i, j}, r });
				}
			}
		}
	}
	else if (node.first <= goal.first and node.second >= goal.second)
	{
		for (int i = node.first; i < goal.first + 1; i++)
		{
			for (int j = node.second; j > goal.second - 1; j--)
			{
				if ((goal.second - node.second - 1) * (i - goal.first) >= (goal.first - node.first + 1) * (j - goal.second))
				{
					double r = sqrt(pow(i - node.first, 2) + pow(j - node.second, 2));
					upperIdx.push_back({ {i, j}, r });
				}
				if ((goal.second - node.second - 1) * (i - goal.first) <= (goal.first - node.first + 1) * (j - goal.second))
				{
					double r = sqrt(pow(i - node.first, 2) + pow(j - node.second, 2));
					lowerIdx.push_back({ {i, j}, r });
				}
			}
		}
	}
	pair<vector<MapIdx>, vector<MapIdx>> idx = { upperIdx, lowerIdx };
	return idx;
}

double heuristic_around_obstacle(p node, p goal, Map map, Map potential_map)
{
	pair<ll, ll> map_size = { map.size() , map[0].size() };
	double r = 3;
	double R = heuristic_e(node, goal);
	vector<p> idx = get_around_index(node, r, map);
	double h = 0;
	for (int i = 0; i < idx.size(); i++)
	{
		switch (map[idx[i].first][idx[i].second])
		{
		case 1:
			h += 1.5 * potential_map[idx[i].first][idx[i].second];
			break;
		case 2:
			h += 4 * potential_map[idx[i].first][idx[i].second];
			break;
		case 3:
			h += 1 * potential_map[idx[i].first][idx[i].second];
			break;
		default:
			break;
		}
	}
	if (idx.size() != 0) {
		h = (h / idx.size()) * sqrt(R);	
	}
	return h;
}

double heuristic(p node, p goal, Map map, Map potential_map)
{
	double h = heuristic_e(node, goal);
	double R = heuristic_e(node, goal);
	double beta = 40;

	if (goal.first - node.first == goal.second - node.second)
	{
		vector<MapIdx> idx = get_index_to_goal_sq(node, goal);
		if (idx.size() == 0)
			return 1;
		double h_add = 0;
		for (int i = 0; i < idx.size(); i++)
		{
			h_add += (1 - pow(min(idx[i].second, R) / (R + 0.01), 3)) * potential_map[idx[i].first.first][idx[i].first.second];
		}
		h *= (h_add / idx.size()) * sqrt(R);
	}
	else
	{
		pair<vector<MapIdx>, vector<MapIdx>> idx = get_index_to_goal_rect(node, goal);
		if (idx.first.size() == 0 || idx.second.size() == 0)
			return 1;
		double h_add1 = 0, h_add2 = 0;
		for (int i = 0; i < idx.first.size(); i++)
		{
			h_add1 += (1 - pow(min(idx.first[i].second, R) / (R + 0.01), 3)) * potential_map[idx.first[i].first.first][idx.first[i].first.second];
		}
		for (int i = 0; i < idx.second.size(); i++)
		{
			h_add2 += (1 - pow(min(idx.second[i].second, R) / (R + 0.01), 3)) * potential_map[idx.second[i].first.first][idx.second[i].first.second];
		}
		h_add1 = (h_add1 / idx.first.size()) * sqrt(R);
		h_add2 = (h_add2 / idx.second.size()) * sqrt(R);
	}

	h += heuristic_around_obstacle(node, goal, map, potential_map);
	return h;
}