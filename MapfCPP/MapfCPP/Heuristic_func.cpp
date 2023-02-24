#include "Heuristic_func.h"
#include <tuple>
#include <cmath>

double heuristic_m(pairInt node, pairInt goal, double d = 1)
{
	int dx = abs(node.first - goal.first);
	int dy = abs(node.second - goal.second);
	return d * (dx + dy);
}

double heuristic_d(pairInt node, pairInt goal, double d1 = 1, double d2 = sqrt(2))
{
	int dx = abs(node.first - goal.first);
	int dy = abs(node.second - goal.second);
	return d1 * (dx + dy) + (d2 - 2 * d1) * min(dx, dy);
}

double heuristic_e(pairInt node, pairInt goal)
{
	int dx = abs(node.first - goal.first);
	int dy = abs(node.second - goal.second);
	return sqrt(dx * dx + dy * dy);
}

double get_heuristic_to_goal_sq(pairInt node, pairInt goal, double R, Map& potential_map)
{
	int len = abs(goal.first - node.first);
	if (!len)
		return 1;

	double h_add = R;
	for (int i = 0; i < abs(goal.first - node.first); i++)
	{
		double x = i + node.first;
		double y = i + node.second;
		double r = sqrt(2) * i + node.first;
		h_add += (1 - pow(min(r, R) / (R + 0.01), 3)) * potential_map[x][y];
	}
	double h = R;
	h *= (h_add / len) * sqrt(R);
	return h;
}

double get_heuristic_to_goal_rect(pairInt node, pairInt goal, double R, Map& potential_map)
{
	double h_add1 = 0, h_add2 = 0;
	ll upperCnt = 0, lowerCnt = 0;
	if (node.first <= goal.first and node.second <= goal.second)
	{
		for (int i = node.first; i < goal.first + 1; i++)
		{
			for (int j = node.second; j < goal.second + 1; j++)
			{
				if ((goal.second - node.second + 1) * (i - goal.first) >= (goal.first - node.first + 1) * (j - goal.second))
				{
					h_add1 += (1 - pow(sqrt(pow(i - node.first, 2) + pow(j - node.second, 2)) / (R + 0.01), 3)) * potential_map[i][j];
					upperCnt++;
				}
				if ((goal.second - node.second + 1) * (i - goal.first) <= (goal.first - node.first + 1) * (j - goal.second))
				{
					h_add2 += (1 - pow(sqrt(pow(i - node.first, 2) + pow(j - node.second, 2)) / (R + 0.01), 3)) * potential_map[i][j];
					lowerCnt++;
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
					h_add1 += (1 - pow(sqrt(pow(i - node.first, 2) + pow(j - node.second, 2)) / (R + 0.01), 3)) * potential_map[i][j];
					upperCnt++;
				}
				if ((goal.second - node.second + 1) * (i - goal.first) <= (goal.first - node.first - 1) * (j - goal.second))
				{
					h_add2 += (1 - pow(sqrt(pow(i - node.first, 2) + pow(j - node.second, 2)) / (R + 0.01), 3)) * potential_map[i][j];
					lowerCnt++;
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
					h_add1 += (1 - pow(sqrt(pow(i - node.first, 2) + pow(j - node.second, 2)) / (R + 0.01), 3)) * potential_map[i][j];
					upperCnt++;
				}
				if ((goal.second - node.second - 1) * (i - goal.first) <= (goal.first - node.first - 1) * (j - goal.second))
				{
					h_add2 += (1 - pow(sqrt(pow(i - node.first, 2) + pow(j - node.second, 2)) / (R + 0.01), 3)) * potential_map[i][j];
					lowerCnt++;
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
					h_add1 += (1 - pow(sqrt(pow(i - node.first, 2) + pow(j - node.second, 2)) / (R + 0.01), 3)) * potential_map[i][j];
					upperCnt++;
				}
				if ((goal.second - node.second - 1) * (i - goal.first) <= (goal.first - node.first + 1) * (j - goal.second))
				{
					h_add2 += (1 - pow(sqrt(pow(i - node.first, 2) + pow(j - node.second, 2)) / (R + 0.01), 3)) * potential_map[i][j];
					lowerCnt++;
				}
			}
		}
	}

	if (!upperCnt || !lowerCnt)
		return R;

	h_add1 = (h_add1 / upperCnt) * sqrt(R);
	h_add2 = (h_add2 / lowerCnt) * sqrt(R);
	double h = min(h_add1, h_add2);

	return h;
}

vector<pairInt> get_around_index(pairInt node, double r, Map& map)
{
	pair<ll, ll> map_size = { map.size() , map[0].size() };
	vector<pairInt> idx;
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

double heuristic_around_obstacle(pairInt node, pairInt goal, Map& map, Map& potential_map)
{
	pair<ll, ll> map_size = { map.size() , map[0].size() };
	double r = 3;
	double R = heuristic_e(node, goal);
	vector<pairInt> idx = get_around_index(node, r, map);
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

double heuristic(pairInt node, pairInt goal, Map& map, Map& potential_map)
{
	double R = heuristic_e(node, goal);
	double h = R;
	double beta = 40;

	if (abs(goal.first - node.first) == abs(goal.second - node.second))
		h = get_heuristic_to_goal_sq(node, goal, R, potential_map);
	else
		h = get_heuristic_to_goal_rect(node, goal, R, potential_map);

	double r = 3;
	h += heuristic_around_obstacle(node, goal, map, potential_map);

	return h;
}
